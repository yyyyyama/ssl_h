set(NNablaExtCUDA_FOUND 0)

# nnabla-ext-cuda と思われるライブラリとインクルードパスを求める
# ライブラリ名は CUDA/cuDNN のバージョンがついている場合も考慮する
# https://docs.nvidia.com/deeplearning/sdk/cudnn-support-matrix/index.html
# TODO: マルチ GPU 版に対応する or 任意のライブラリ名を渡せるようにする
set(libnnabla_cuda_names nnabla_cuda)
foreach(cuda_version IN ITEMS 113 112 111 110 102 101 100 92 90 80)
  foreach(cudnn_version IN ITEMS 8 7)
    list(APPEND libnnabla_cuda_names "nnabla_cuda${cuda_version}_${cudnn_version}")
  endforeach()
endforeach()
find_library(libnnabla_cuda NAMES ${libnnabla_cuda_names})
find_path(nbla_cuda_dir nbla/cuda/init.hpp)

# 見つかったライブラリで nnabla-ext-cuda を使ったコードをコンパイルしてみる
if(libnnabla_cuda AND nbla_cuda_dir)
  set(bindir "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/nnabla-ext-cuda")
  set(test_exe  "${bindir}/print_nbla_cuda_version")
  set(test_file "${test_exe}.cc")
  file(WRITE "${test_file}"
    "#include <iostream>\n"
    "#include <${nbla_cuda_dir}/nbla/cuda/init.hpp>\n"
    "namespace nbla { const std::string nbla_ext_cuda_version(void); }\n"
    "int main() { std::cout << nbla::nbla_ext_cuda_version(); }")
  try_compile(build_success "${bindir}" "${test_file}"
    COPY_FILE       "${test_exe}"
    LINK_LIBRARIES  "${libnnabla_cuda}")
endif()

if(build_success)
  execute_process(COMMAND "${test_exe}"
    OUTPUT_VARIABLE pnv_output
    ERROR_VARIABLE  pnv_error
    RESULT_VARIABLE pnv_result)

  # ビルド・実行に成功したら確定
  if(${pnv_result} EQUAL 0)
    set(NNablaExtCUDA_FOUND 1)
    set(NNablaExtCUDA_VERSION     "${pnv_output}")
    set(NNablaExtCUDA_LIBRARIES   "${libnnabla_cuda}")
    set(NNablaExtCUDA_INCLUDE_DIR "${nbla_cuda_dir}")
  endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(NNablaExtCUDA
  REQUIRED_VARS NNablaExtCUDA_LIBRARIES NNablaExtCUDA_INCLUDE_DIR
  VERSION_VAR   NNablaExtCUDA_VERSION)

mark_as_advanced(
  NNablaExtCUDA_LIBRARIES
  NNablaExtCUDA_INCLUDE_DIR)

if(NNablaExtCUDA_FOUND AND NOT TARGET nnabla::nnabla_cuda)
  add_library(nnabla::nnabla_cuda UNKNOWN IMPORTED)
  set_target_properties(nnabla::nnabla_cuda PROPERTIES
    IMPORTED_LOCATION "${NNablaExtCUDA_LIBRARIES}"
    INTERFACE_INCLUDE_DIRECTORIES "${NNablaExtCUDA_INCLUDE_DIR}")
endif()
