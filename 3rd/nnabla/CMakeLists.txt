# nnabla の Python 検出が微妙なので、先にこっちで見つけておく
# https://cmake.org/cmake/help/v3.16/module/FindPython3.html
find_package(Python3 COMPONENTS Interpreter REQUIRED)
set(PYTHON_COMMAND_NAME  "${Python3_EXECUTABLE}" CACHE STRING "Python3 executable path")

# C++ のユーティリティをビルドする
set(BUILD_CPP_UTILS ON CACHE BOOL "Build C++ API and utilities")

# Python インターフェースはビルドしない
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
