#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include <infiniband/verbs.h>
#include "hashpipe_ibverbs.h"

// Alignment size to use.  Currently set to 64 (== 512/8) for compatibility
// with AVX512 instructions.
#define PKT_ALIGNMENT_SIZE (64)

// Maximum number of chunks supported
#define MAX_CHUNKS (8)

// Structure that holds info about a "chunk".  A chunk is part of a packet that
// is stored at a PKT_ALIGNMENT_SIZE aligned address.  The chunk_size is the
// number of bytes from the packet that are stored in the chunk.  The
// chunk_aligned_size is chunk_size rounded up to the next multple of
// PKT_ALIGNMENT_SIZE.  The chunk_offset is the offset of the chunk from the
// packet's first chunk.  The first chunk will have a chunk_offset of 0.
struct hpguppi_pktbuf_chunk {
  size_t chunk_size;
  size_t chunk_aligned_size;
  off_t chunk_offset;
};

// Structure that holds info about packet/slot/block sizing.  A block is
// divided into "slots".  Each slot holds one packet, possibly with internal
// and/or trailing padding added to align various sections of the packet to
// PKT_ALIGNMENT_SIZE.  These sections are called chunks.  num_chunks specifies
// the number of chunks that are being used.  The pkt_size is the sum of the
// (unaligned) sizes of all chunks.  The slot_size is the size of a slot and
// equals the sum of the chunk_aligned_sizes.  slots_per_block is the number of
// slots in a data block.  Note that slot_size * slots_per_block may be less
// than the size of data block by up PKT_ALIGNMENT_SIZE-1 bytes.
struct hpguppi_pktbuf_info {
  uint32_t num_chunks;
  size_t pkt_size;
  size_t slot_size;
  size_t slots_per_block;
  struct hpguppi_pktbuf_chunk chunks[MAX_CHUNKS];
};

#define BLOCK_DATA_SIZE (128*8192)


// Queries the device specified by interface_name and returns max_qp_wr, or -1
// on error.
int
query_max_wr(const char * interface_name)
{
  uint64_t interface_id;
  struct ibv_device_attr* ibv_dev_attr = malloc(sizeof(struct ibv_device_attr));
  struct ibv_context* ibv_context = NULL;
  int max_qp_wr = -1;

  if(hashpipe_ibv_get_interface_info(interface_name, NULL, &interface_id)) {
    fprintf(stderr, "%s: error getting interace info for %s\n", __FUNCTION__, interface_name);
    errno = 0;
    return -1;
  }

  if(hashpipe_ibv_open_device_for_interface_id(
        interface_id, &ibv_context, ibv_dev_attr, NULL)) {
    // Error message already logged
    return -1;
  }

  max_qp_wr = ibv_dev_attr->max_qp_wr;
  free(ibv_dev_attr);
  return max_qp_wr;
}

// The hpguppi_ibverbs_init() function sets up the hashpipe_ibv_context
// structure and then call hashpipe_ibv_init().  This uses the "user-managed
// buffers" feature of hashpipe_ibverbs so that packets will be stored directly
// into the data blocks of the shared memory databuf.  It initializes receive
// scatter/gather lists to point to slots in the first data block of the shared
// memory databuf.  Due to how the current hashpipe versions works, the virtual
// address mappings for the shared memory datbuf change between ini() and
// run(), so this function must be called from run() only.  It initializes
// receive scatter/gather lists to point to slots in the first data block of
// the shared memory databuf.  Returns HASHPIPE_OK on success, other values on
// error.
int
ibverbs_init(
	struct hashpipe_ibv_context* hibv_ctx,
	struct hpguppi_pktbuf_info* pktbuf_info,
  uint8_t* buffer,
  size_t buffer_byte_size,
	char* interface_name,
	int max_flows
) {
  int i, j;
  uint32_t num_chunks = pktbuf_info->num_chunks;
  struct hpguppi_pktbuf_chunk * chunks = pktbuf_info->chunks;
  uint64_t base_addr;

  memset(hibv_ctx, 0, sizeof(struct hashpipe_ibv_context));

  // MAXFLOWS got initialized by init() if needed, but we setup the default
  // value again just in case some (buggy) downstream thread removed it from
  // the status buffer.
  hibv_ctx->max_flows = max_flows;
	strcpy(hibv_ctx->interface_name, interface_name);

  // General fields
  hibv_ctx->nqp = 1;
  hibv_ctx->pkt_size_max = pktbuf_info->slot_size; // max for both send and receive //not pkt_size as it is used for the cumulative sge buffers
  hibv_ctx->user_managed_flag = 1;

  // Number of send/recv packets (i.e. number of send/recv WRs)
  hibv_ctx->send_pkt_num = 1;
  int num_recv_wr = query_max_wr(hibv_ctx->interface_name);
  fprintf(stderr, "%s: max work requests of %s = %d\n", __FUNCTION__, hibv_ctx->interface_name, num_recv_wr);
  if(num_recv_wr > pktbuf_info->slots_per_block) {
    num_recv_wr = pktbuf_info->slots_per_block;
  }
  hibv_ctx->recv_pkt_num = num_recv_wr;
  fprintf(stderr, "%s: using %d work requests\n", __FUNCTION__, num_recv_wr);

  if(hibv_ctx->recv_pkt_num * hibv_ctx->pkt_size_max > BLOCK_DATA_SIZE){
    // Should never happen
    fprintf(stderr, "%s: hibv_ctx->recv_pkt_num (%u)*(%u) hibv_ctx->pkt_size_max (%u) > (%u) BLOCK_DATA_SIZE\n", __FUNCTION__,
    hibv_ctx->recv_pkt_num, hibv_ctx->pkt_size_max, hibv_ctx->recv_pkt_num * hibv_ctx->pkt_size_max, BLOCK_DATA_SIZE);
  }

  // Allocate packet buffers
  if(!(hibv_ctx->send_pkt_buf = (struct hashpipe_ibv_send_pkt *)calloc(
      hibv_ctx->send_pkt_num, sizeof(struct hashpipe_ibv_send_pkt)))) {
    return 1;
  }
  if(!(hibv_ctx->recv_pkt_buf = (struct hashpipe_ibv_recv_pkt *)calloc(
      hibv_ctx->recv_pkt_num, sizeof(struct hashpipe_ibv_recv_pkt)))) {
    return 1;
  }

  // Allocate sge buffers.  We allocate num_chunks SGEs per receive WR.
  if(!(hibv_ctx->send_sge_buf = (struct ibv_sge *)calloc(
      hibv_ctx->send_pkt_num, sizeof(struct ibv_sge)))) {
    return 1;
  }
  if(!(hibv_ctx->recv_sge_buf = (struct ibv_sge *)calloc(
      hibv_ctx->recv_pkt_num * num_chunks, sizeof(struct ibv_sge)))) {
    return 1;
  }

  // Specify size of send and recv memory regions.
  // Send memory region is just one packet.  Recv memory region spans a data block, with
  // one recv memory region registered per block (see recv_mr_num).
  hibv_ctx->send_mr_size = (size_t)hibv_ctx->send_pkt_num * hibv_ctx->pkt_size_max;
  hibv_ctx->recv_mr_size = buffer_byte_size;

  // Allocate memory for send_mr_buf
  if(!(hibv_ctx->send_mr_buf = (uint8_t *)calloc(
      hibv_ctx->send_pkt_num, hibv_ctx->pkt_size_max))) {
    return 1;
  }
  // Point recv_mr_buf to starts of block 0
  hibv_ctx->recv_mr_buf = buffer;

  // Setup send WR's num_sge and SGEs' addr/length fields
  hibv_ctx->send_pkt_buf[0].wr.num_sge = 1;
  hibv_ctx->send_pkt_buf[0].wr.sg_list = hibv_ctx->send_sge_buf;
  hibv_ctx->send_sge_buf[0].addr = (uint64_t)hibv_ctx->send_mr_buf;
  hibv_ctx->send_sge_buf[0].length = hibv_ctx->pkt_size_max;

  // Setup recv WRs' num_sge and SGEs' addr/length fields
  for(i=0; i<hibv_ctx->recv_pkt_num; i++) {
    hibv_ctx->recv_pkt_buf[i].wr.wr_id = i;
    hibv_ctx->recv_pkt_buf[i].wr.num_sge = num_chunks;
    hibv_ctx->recv_pkt_buf[i].wr.sg_list = &hibv_ctx->recv_sge_buf[num_chunks*i];

  	base_addr = buffer + i * pktbuf_info->slot_size;
    for(j=0; j<num_chunks; j++) {
      hibv_ctx->recv_sge_buf[num_chunks*i+j].addr = base_addr + chunks[j].chunk_offset;
      hibv_ctx->recv_sge_buf[num_chunks*i+j].length = chunks[j].chunk_size;
    }
  }

  // Initialize ibverbs
  return hashpipe_ibv_init(hibv_ctx);
}

// Create MAC-sniffer flow
// Use with caution!!!
struct ibv_flow *
create_macsniffer_flow(struct hashpipe_ibv_context * hibv_ctx)
{
  // hashpipe_ibv_flow(hibv_ctx,
  //   hibv_ctx->max_flows-1, IBV_FLOW_SPEC_UDP,
  //   hibv_ctx->mac,    NULL,
  //   0, 0,
  //   0, 0,
  //   0, 0);
  // return hibv_ctx->ibv_flows[hibv_ctx->max_flows-1];
  struct hashpipe_ibv_flow flow = {
    .attr = {
      .comp_mask      = 0,
      .type           = IBV_FLOW_ATTR_NORMAL,
      .size           = sizeof(flow.attr)
                        + sizeof(struct ibv_flow_spec_ipv4)
                        + sizeof(struct ibv_flow_spec_eth)
                        + sizeof(struct ibv_flow_spec_tcp_udp),
      .priority       = 0,
      .num_of_specs   = 3,
      .port           = hibv_ctx->port_num,
      .flags          = 0
    },
    .spec_eth = {
      .type   = IBV_FLOW_SPEC_ETH,
      .size   = sizeof(flow.spec_eth),
    },
    .spec_ipv4 = {
      .type   = IBV_FLOW_SPEC_IPV4,
      .size   = sizeof(flow.spec_ipv4),
    },
    .spec_tcp_udp = {
      .type   = IBV_FLOW_SPEC_UDP,
      .size   = sizeof(flow.spec_tcp_udp),
      .val.dst_port = htobe16(4179),
      .mask.dst_port = 0xffff,
    }
  };

  memcpy(flow.spec_eth.val.dst_mac, hibv_ctx->mac, 6);
  memset(flow.spec_eth.mask.dst_mac, 0xff, 6);

  return ibv_create_flow(hibv_ctx->qp[0], (struct ibv_flow_attr *) &flow);
}


// Parses the ibvpktsz string for chunk sizes and initializes db's pktbuf_info
// accordingly.  Returns 0 on success or -1 on error.
int
parse_ibvpktsz(struct hpguppi_pktbuf_info *pktbuf_info, char * ibvpktsz)
{
  int i;
  char * p;
  uint32_t nchunks = 0;
  size_t pkt_size = 0;
  size_t slot_size = 0;

  if(!ibvpktsz) {
    return -1;
  }

  // Look for commas
  while(nchunks < MAX_CHUNKS && (p = strchr(ibvpktsz, ','))) {
    // Replace comma with nul
    *p = '\0';
    // Parse chuck size
    pktbuf_info->chunks[nchunks].chunk_size = strtoul(ibvpktsz, NULL, 0);
    // Replace nul with comma
    *p = ',';
    // If chunk_size is 0, return error
    if(pktbuf_info->chunks[nchunks].chunk_size == 0) {
      fprintf(stderr, "%s: IBVPKTSZ chunk size must be non-zero\n", __FUNCTION__);
      return -1;
    }
    // Increment nchunks
    nchunks++;
    // Advance ibvpktsz to character beyond p
    ibvpktsz = p+1;
  }

  // If nchunks is less than MAX_CHUNKS and ibvpktsz[0] is not nul
  if(nchunks < MAX_CHUNKS && *ibvpktsz) {
    // If more commas remain, too many chunks!
    if(strchr(ibvpktsz, ',')) {
      fprintf(stderr, "%s: IBVPKTSZ has too many chunks\n", __FUNCTION__);
      return -1;
    }
    // Parse final chunk size
    pktbuf_info->chunks[nchunks].chunk_size = strtoul(ibvpktsz, NULL, 0);
    // Increment nchunks
    nchunks++;
  } else if(nchunks == MAX_CHUNKS && *ibvpktsz) {
    // Too many chunks
    fprintf(stderr, "%s: IBVPKTSZ has too many chunks\n", __FUNCTION__);
    return -1;
  }

  // Calculate remaining fields
  for(i=0; i<nchunks; i++) {
    pktbuf_info->chunks[i].chunk_aligned_size = pktbuf_info->chunks[i].chunk_size +
      ((-pktbuf_info->chunks[i].chunk_size) % PKT_ALIGNMENT_SIZE);
    pktbuf_info->chunks[i].chunk_offset = slot_size;
    // Accumulate pkt_size and slot_size
    pkt_size += pktbuf_info->chunks[i].chunk_size;
    slot_size += pktbuf_info->chunks[i].chunk_aligned_size;
  }

  // Store final values
  pktbuf_info->num_chunks = nchunks;
  pktbuf_info->pkt_size = pkt_size;
  pktbuf_info->slot_size = slot_size;
  pktbuf_info->slots_per_block = BLOCK_DATA_SIZE / slot_size;

  pktbuf_info->slots_per_block -= pktbuf_info->slots_per_block % 12;

  return 0;
}

int main() {
  char interface_name[] = "enp225s0f1";
  char ibvpktsz[] = "42,12,8,4096";
	struct hpguppi_pktbuf_info pktbuf_info;

	if(parse_ibvpktsz(&pktbuf_info, ibvpktsz)){
		fprintf(stderr, "failed to parse_ibvpktsz\n");
		return 1;
	}

	uint8_t* buffer = malloc(BLOCK_DATA_SIZE);

	struct hashpipe_ibv_context hibv_ctx = {0};
	ibverbs_init(
		&hibv_ctx,
		&pktbuf_info,
		buffer, BLOCK_DATA_SIZE,
		interface_name,
		1
	);

  struct ibv_flow * sniffer_flow = NULL;
  if(!(sniffer_flow = create_macsniffer_flow(&hibv_ctx))) {
      fprintf(stderr, "create_macsniffer_flow failed\n");
      return 1;
  }
  
  // Variables for handing received packets
  struct hashpipe_ibv_recv_pkt * hibv_rpkt = NULL;
  struct hashpipe_ibv_recv_pkt * curr_rpkt;

  // Main loop
  while (1) {
    hibv_rpkt = hashpipe_ibv_recv_pkts(&hibv_ctx, 50); // 50 ms timeout

    // If no packets and errno is non-zero
    if(!hibv_rpkt && errno) {
      // Print error, reset errno, and continue receiving
      fprintf(stderr, "hashpipe_ibv_recv_pkts failure\n");
      errno = 0;
      continue;
    }

    printf("Received packets\n");
    // For each packet
    for(curr_rpkt = hibv_rpkt; curr_rpkt;
        curr_rpkt = (struct hashpipe_ibv_recv_pkt *)curr_rpkt->wr.next) {
      
    }
    
    // Release packets (i.e. repost work requests)
    if(hashpipe_ibv_release_pkts(&hibv_ctx,
          (struct hashpipe_ibv_recv_pkt *)hibv_rpkt)) {
      fprintf(stderr, "hashpipe_ibv_release_pkts failure\n");
      errno = 0;
    }
    break;
  }

  // Cleanup
  ibv_destroy_flow(sniffer_flow);
  hashpipe_ibv_shutdown(&hibv_ctx);
	
	free(buffer);
	return 0;
}