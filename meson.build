project('RoCE_PoC', 'c',
  version: '0.1',
)

cc = meson.get_compiler('c')

hashpipe_inc = include_directories('./external/hashpipe/src/')
hashpipe_dep = cc.find_library(
  'hashpipe_ibverbs',
  dirs : [meson.current_source_dir()+'/external/hashpipe/src/.libs'],
  has_headers: ['hashpipe_ibverbs.h']
)
cc.has_header_symbol(
  'hashpipe_ibverbs.h',
  'hashpipe_ibv_init'
)

src_lst = []
inc_lst = [
	include_directories('include'),
  hashpipe_inc
]
dep_lst = [
  hashpipe_dep,
]

m_dep = cc.find_library('m', required : true)
dep_lst += [
  m_dep
]

subdir('include')
subdir('src')

lib_roce_dep = declare_dependency(
  include_directories: inc_lst,
  dependencies: dep_lst
)

subdir('tests')
