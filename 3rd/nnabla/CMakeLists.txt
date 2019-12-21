# nnabla の Python 検出が微妙なので、先にこっちで見つけておく
# https://cmake.org/cmake/help/v3.16/module/FindPython3.html
find_package(Python3 COMPONENTS Interpreter REQUIRED)

# ExternalProject で nnabla をビルドする
# https://cmake.org/cmake/help/v3.16/module/ExternalProject.html
include(ExternalProject)
ExternalProject_Add(nnabla-ext
  GIT_REPOSITORY https://github.com/sony/nnabla.git
  GIT_TAG        v1.4.0
  STEP_TARGETS   build
  CMAKE_CACHE_ARGS
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_CURRENT_BINARY_DIR}/nnabla
    -DPYTHON_COMMAND_NAME:STRING=${Python3_EXECUTABLE}
    -DBUILD_CPP_UTILS:BOOL=ON
    -DBUILD_PYTHON_PACKAGE:BOOL=OFF
  BUILD_BYPRODUCTS
    ${CMAKE_CURRENT_BINARY_DIR}/nnabla/lib/libnbla_cli.so
    ${CMAKE_CURRENT_BINARY_DIR}/nnabla/lib/libnnabla.so
    ${CMAKE_CURRENT_BINARY_DIR}/nnabla/lib/libnnabla_utils.so
)

# ビルドした nnabla を扱いやすくするための宣言
add_library(nnabla INTERFACE)
add_library(nnabla::nnabla ALIAS nnabla)
target_link_libraries(nnabla
  INTERFACE ${CMAKE_CURRENT_BINARY_DIR}/nnabla/lib/libnnabla.so
)
target_include_directories(nnabla SYSTEM
  INTERFACE ${CMAKE_CURRENT_BINARY_DIR}/nnabla/include
)
add_dependencies(nnabla nnabla-ext)

add_library(nnabla_utils INTERFACE)
add_library(nnabla::nnabla_utils ALIAS nnabla_utils)
target_link_libraries(nnabla_utils
  INTERFACE ${CMAKE_CURRENT_BINARY_DIR}/nnabla/lib/libnnabla_utils.so nnabla
)

add_library(nbla_cli INTERFACE)
add_library(nnabla::nbla_cli ALIAS nbla_cli)
target_link_libraries(nbla_cli
  INTERFACE ${CMAKE_CURRENT_BINARY_DIR}/nnabla/lib/libnbla_cli.so nnabla nnabla_utils
)
