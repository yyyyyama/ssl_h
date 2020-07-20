cmake_minimum_required(VERSION 3.13)

project(ai-server CXX)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

# Disable in-source build
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES  ON)

# Set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# 基本的なコンパイルオプションを設定
# これらは ai-server 以外 (3rd/ など) にも影響するため最小限 (最適化関連など) にとどめる
set(CMAKE_CXX_EXTENSIONS  OFF)
set(CMAKE_CXX_STANDARD    17)
set(CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS}  -Wl,-O1,--sort-common,--as-needed,-z,relro")

# ai-server 関連のコードのコンパイラオプションの設定
add_library(ai-server-common-flags INTERFACE)
target_compile_options(ai-server-common-flags INTERFACE
  # 警告を少し厳しめに
  $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -pedantic>
)
target_compile_definitions(ai-server-common-flags INTERFACE
  # Debug ビルド時に AI_SERVER_DEBUG を define
  $<$<AND:$<COMPILE_LANGUAGE:CXX>,$<CONFIG:Debug>>:AI_SERVER_DEBUG>
)

# Ninja を使うとコンパイラのカラー表示が無効化されてしまうので有効にする
if(CMAKE_GENERATOR STREQUAL "Ninja")
  set(color_diag_options
    $<$<AND:$<COMPILE_LANGUAGE:CXX>,$<CXX_COMPILER_ID:GNU>>:-fdiagnostics-color=always>
    $<$<AND:$<COMPILE_LANGUAGE:CXX>,$<CXX_COMPILER_ID:Clang>>:-fcolor-diagnostics>
  )
  target_compile_options(ai-server-common-flags INTERFACE ${color_diag_options})
  target_link_options(ai-server-common-flags INTERFACE ${color_diag_options})
endif()

# Sanitizers
if(ENABLE_SANITIZERS)
  set(sanitiser_options
    $<$<AND:$<COMPILE_LANGUAGE:CXX>,$<CXX_COMPILER_ID:GNU>>:-fsanitize=address>
    $<$<AND:$<COMPILE_LANGUAGE:CXX>,$<CXX_COMPILER_ID:Clang>>:-fsanitize=address -fno-omit-frame-pointer>
  )
  target_compile_options(ai-server-common-flags INTERFACE ${sanitiser_options})
  target_link_options(ai-server-common-flags INTERFACE ${sanitiser_options})
endif()

# Code coverage
if(ENABLE_CODE_COVERAGE)
  set(coverage_options
    $<$<AND:$<COMPILE_LANGUAGE:CXX>,$<CXX_COMPILER_ID:GNU>>:-fprofile-arcs -ftest-coverage>
    $<$<AND:$<COMPILE_LANGUAGE:CXX>,$<CXX_COMPILER_ID:Clang>>:-fprofile-instr-generate -fcoverage-mapping>
  )
  target_compile_options(ai-server-common-flags INTERFACE ${coverage_options})
  target_link_options(ai-server-common-flags INTERFACE ${coverage_options})
endif()

# Required libraries
find_package(Boost 1.66.0 COMPONENTS coroutine system unit_test_framework REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(Protobuf 3.2 REQUIRED)
find_package(Threads REQUIRED)

option(USE_SYSTEM_NNABLA "try to use system installed nnabla" OFF)
option(ENABLE_NNABLA_EXT_CUDA "enable cuda extension of nnabla" OFF)
if(USE_SYSTEM_NNABLA)
  find_package(NNabla 1.4 COMPONENTS NNabla Utils)

  # インストールされた nnabla が見つかったときは nnabla-ext-cuda もインストールされたものを使う
  # (nnabla-ext-cuda のビルドには nnabla のソースコードが必要で、単体でのビルドが難しいため)
  if(NNabla_FOUND AND ENABLE_NNABLA_EXT_CUDA)
    find_package(NNablaExtCUDA 1.4 REQUIRED)
  endif()
endif()

# Enable CTest
enable_testing()

add_subdirectory(3rd)
add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(app)
