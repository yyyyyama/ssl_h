file(GLOB_RECURSE SOURCES ./*.cc)
add_library(ai-server-lib ${SOURCES})
target_link_libraries(ai-server-lib PRIVATE ai-server-common-flags)
target_link_libraries(ai-server-lib PUBLIC
  Boost::boost
  Boost::coroutine
  Boost::system
  Eigen3::Eigen
  Threads::Threads
  fmt::fmt
  nnabla::nnabla
  nnabla::nnabla_utils
  protobuf::libprotobuf
  ssl-protos::ssl-protos
  yamlizer::yamlizer
)
target_include_directories(ai-server-lib PUBLIC ${PROJECT_SOURCE_DIR}/src)
set_target_properties(ai-server-lib PROPERTIES OUTPUT_NAME ai-server)
