# nnabla の Python 検出が微妙なので、先にこっちで見つけておく
# https://cmake.org/cmake/help/v3.16/module/FindPython3.html
find_package(Python3 COMPONENTS Interpreter REQUIRED)
set(PYTHON_COMMAND_NAME  "${Python3_EXECUTABLE}" CACHE STRING "Python3 executable path")

# C++ のユーティリティをビルドする (nnabla, nnabla-cuda-ext)
set(BUILD_CPP_UTILS ON CACHE BOOL "Build C++ API and utilities")

# C++ のライブラリをビルドする (nnabla-cuda-ext)
set(BUILD_CPP_LIB ON CACHE BOOL "Build C++ Library")

# Python インターフェースはビルドしない (nnabla, nnabla-cuda-ext)
set(BUILD_PYTHON_PACKAGE OFF CACHE BOOL "Build python package")

# patch を使いたい
find_package(Patch REQUIRED)

# FetchContent で nnabla のソースコードをダウンロード & プロジェクトへの追加をする
# https://cmake.org/cmake/help/v3.13/module/FetchContent.html
include(FetchContent)
FetchContent_Declare(nnabla
  URL      https://github.com/sony/nnabla/archive/v1.7.0.tar.gz
  URL_HASH SHA256=7602b95a547a0c593cbc040cfc771ea60619c5128748722533ac2fb48a150c9d

  # nnabla の CMakeLists.txt 内では CMAKE_SOURCE_DIR などが使われていて
  # add_subdirectory/FetchContent_MakeAvailable したときに問題となるためパッチを当てる
  PATCH_COMMAND ${Patch_EXECUTABLE} -p1 < ${CMAKE_CURRENT_SOURCE_DIR}/use-correct-variables.patch
)
FetchContent_GetProperties(nnabla)
if(NOT nnabla_POPULATED)
  FetchContent_Populate(nnabla)
  add_subdirectory(${nnabla_SOURCE_DIR} ${nnabla_BINARY_DIR})
endif()

# nnabla 側で宣言されたターゲット nnabla, nnabla_utils, nnabla_cli に
# 追加の依存関係と include パスの指定をして使いやすくする
add_library(nnabla_interface INTERFACE)
target_link_libraries(nnabla_interface INTERFACE nnabla)
target_include_directories(nnabla_interface SYSTEM INTERFACE "${nnabla_SOURCE_DIR}/include")
add_library(nnabla::nnabla ALIAS nnabla_interface)

add_library(nnabla_utils_interface INTERFACE)
target_link_libraries(nnabla_utils_interface INTERFACE
  nnabla::nnabla
  nnabla_utils
)
add_library(nnabla::nnabla_utils ALIAS nnabla_utils_interface)

add_library(nnabla_cli_interface INTERFACE)
target_link_libraries(nnabla_cli_interface INTERFACE
  nnabla::nnabla
  nnabla::nnabla_utils
  nnabla_cli
)
add_library(nnabla::nnabla_cli ALIAS nnabla_cli_interface)

if(ENABLE_NNABLA_EXT_CUDA)
  set(NNABLA_DIR "${nnabla_SOURCE_DIR}")
  set(NBLA_LIBRARY_NAME nnabla::nnabla)
  set(NBLA_UTILS_LIBRARY_NAME nnabla::nnabla_utils)
  set(NBLA_CLI_LIBRARY_NAME nnabla::nnabla_cli)

  FetchContent_Declare(nnabla_ext_cuda
    URL      https://github.com/sony/nnabla-ext-cuda/archive/v1.7.0.tar.gz
    URL_HASH SHA256=b722012eecc783509da822c301af6e7129b2dd714d3a2b3f174a15b72d7075d8

    # - CMAKE_SOURCE_DIR を PROJECT_SOURCE_DIR に変更
    # - "存在する" libnnabla.so などのパスを要求してきて不便なのを修正
    # - Eigen をダウンロードしてくるのをやめる
    PATCH_COMMAND ${Patch_EXECUTABLE} -p1 < ${CMAKE_CURRENT_SOURCE_DIR}/nnabla-ext-cuda-v1.7.0.patch
  )
  FetchContent_GetProperties(nnabla_ext_cuda)
  if(NOT nnabla_ext_cuda_POPULATED)
    FetchContent_Populate(nnabla_ext_cuda)
    add_subdirectory(${nnabla_ext_cuda_SOURCE_DIR} ${nnabla_ext_cuda_BINARY_DIR})
  endif()

  add_library(nnabla_cuda_interface INTERFACE)
  target_include_directories(nnabla_cuda_interface SYSTEM INTERFACE "${nnabla_ext_cuda_SOURCE_DIR}/include")
  target_link_libraries(nnabla_cuda_interface INTERFACE
    nnabla::nnabla
    nnabla_cuda
  )
  add_library(nnabla::nnabla_cuda ALIAS nnabla_cuda_interface)
endif()
