set(NNabla_FOUND 0)

# nnabla と思われるライブラリとインクルードパスを求める
find_library(libnnabla       NAMES nnabla)
find_library(libnnabla_utils NAMES nnabla_utils)
find_library(libnnabla_cli   NAMES nnabla_cli nbla_cli)

find_path(nbla_dir       nbla/version.hpp)
find_path(nbla_utils_dir nbla_utils/nnp.hpp)
find_path(nbla_cli_dir   nbla_cli/nbla_cli.hpp)

# 見つかったライブラリで nnabla を使ったコードをコンパイルしてみる
if(libnnabla AND nbla_dir)
  set(bindir "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/nnabla")
  set(test_file "${bindir}/print_nbla_version.cc")
  file(WRITE "${test_file}"
    "#include <iostream>\n"
    "#include <${nbla_dir}/nbla/version.hpp>\n"
    "int main() { std::cout << nbla::nbla_version(); }")
  try_compile(build_success "${bindir}" "${test_file}"
    COPY_FILE       ${bindir}/print_nbla_version
    LINK_LIBRARIES  "${libnnabla}")
endif()

if(build_success)
  execute_process(COMMAND ${bindir}/print_nbla_version
    OUTPUT_VARIABLE pnv_output
    ERROR_VARIABLE  pnv_error
    RESULT_VARIABLE pnv_result)

  if(${pnv_result} EQUAL 0)
    set(NNabla_FOUND 1)
    set(NNabla_VERSION "${pnv_output}")

    set(NNabla_LIBRARIES)
    set(NNabla_INCLUDE_DIR)

    set(NNabla_NNabla_FOUND TRUE)
    set(NNabla_NNabla_LIB   ${libnnabla})
    list(APPEND NNabla_LIBRARIES   ${libnnabla})
    list(APPEND NNabla_INCLUDE_DIR ${nbla_dir})

    if(libnnabla_utils AND "Utils" IN_LIST NNabla_FIND_COMPONENTS)
      set(NNabla_Utils_FOUND TRUE)
      set(NNabla_Utils_LIB   ${libnnabla_utils})
      list(APPEND NNabla_LIBRARIES   ${libnnabla_utils})
      list(APPEND NNabla_INCLUDE_DIR ${nbla_utils_dir})
    endif()

    if(libnnabla_cli AND "CLI" IN_LIST NNabla_FIND_COMPONENTS)
      set(NNabla_CLI_FOUND TRUE)
      set(NNabla_CLI_LIB   ${libnnabla_cli})
      list(APPEND NNabla_LIBRARIES   ${libnnabla_cli})
      list(APPEND NNabla_INCLUDE_DIR ${nbla_cli_dir})
    endif()
  endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(NNabla
  REQUIRED_VARS NNabla_LIBRARIES NNabla_INCLUDE_DIR
  VERSION_VAR   NNabla_VERSION
  HANDLE_COMPONENTS)

mark_as_advanced(
  NNabla_LIBRARIES
  NNabla_INCLUDE_DIR
  NNabla_NNabla_LIB
  NNabla_Utils_LIB
  NNabla_CLI_LIB)

if(NNabla_FOUND AND NNabla_NNabla_FOUND AND NOT TARGET nnabla::nnabla)
  add_library(nnabla::nnabla UNKNOWN IMPORTED)
  set_target_properties(nnabla::nnabla PROPERTIES
    IMPORTED_LOCATION "${NNabla_NNabla_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${NNabla_INCLUDE_DIR}")
endif()

if(NNabla_FOUND AND NNabla_Utils_FOUND AND NOT TARGET nnabla::nnabla_utils)
  add_library(nnabla::nnabla_utils UNKNOWN IMPORTED)
  set_target_properties(nnabla::nnabla_utils PROPERTIES
    IMPORTED_LOCATION "${NNabla_Utils_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${NNabla_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES nnabla::nnabla)
endif()

if(NNabla_FOUND AND NNabla_CLI_FOUND AND NOT TARGET nnabla::nnabla_cli)
  add_library(nnabla::nnabla_cli UNKNOWN IMPORTED)
  set_target_properties(nnabla::nnabla_cli PROPERTIES
    IMPORTED_LOCATION "${NNabla_CLI_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${NNabla_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "nnabla::nnabla;nnabla::nnabla_utils")
endif()

# 見つかった nnabla_utils が HDF5 をサポートされているか確認する
if(TARGET nnabla::nnabla_utils)
  set(hdf5_supported 0)

  # 空の .h5 ファイルを Nnp に追加したときにそれっぽい例外が飛んだら HDF5 未対応とする
  set(test_h5_file "${bindir}/check_hdf5_support.h5")
  file(WRITE "${test_h5_file}" "")
  set(test_file "${bindir}/check_hdf5_support.cc")
  file(WRITE "${test_file}"
    "#include <cstring>\n"
    "#include <stdexcept>\n"
    "#include <nbla/context.hpp>\n"
    "#include <nbla_utils/nnp.hpp>\n"
    "auto main() -> int {\n"
    "  const nbla::Context ctx{{\"cpu:float\"}, \"CpuCachedArray\", \"0\"};\n"
    "  nbla::utils::nnp::Nnp nnp{ctx};\n"
    "  try {\n"
    "    nnp.add(\"${test_h5_file}\");\n"
    "  } catch (const std::exception& e) {\n"
    "    if (std::strstr(e.what(), \"HDF5\") && std::strstr(e.what(), \"not enabled\")) {\n"
    "      return 1;\n"
    "    }\n"
    "  }\n"
    "}")
  try_compile(build_success "${bindir}" "${test_file}"
    COPY_FILE       ${bindir}/check_hdf5_support
    LINK_LIBRARIES  nnabla::nnabla_utils)
  if(build_success)
    execute_process(COMMAND ${bindir}/check_hdf5_support
      WORKING_DIRECTORY "${bindir}"
      OUTPUT_VARIABLE chs_output
      ERROR_VARIABLE  chs_error
      RESULT_VARIABLE chs_result)
    if(${chs_result} EQUAL 0)
      set(hdf5_supported 1)
    endif()
  endif()

  if(NOT hdf5_supported)
    message(WARNING "HDF5 support is disabled. Runtime errors might be caused when loading newer nnp files.")
  endif()
endif()
