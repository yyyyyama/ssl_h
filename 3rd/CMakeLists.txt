# Suppress all warnings from this directory
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

add_subdirectory(fmt)

# nnabla が見つかっていなかったらビルドする
if(NOT NNabla_FOUND)
  add_subdirectory(nnabla)
endif()

add_subdirectory(ssl-protos)

set(YAMLIZER_BUILD_EXAMPLES   OFF CACHE BOOL "Build example files")
set(YAMLIZER_BUILD_UNIT_TESTS OFF CACHE BOOL "Build unit tests")
add_subdirectory(yamlizer)
