set(NNabla_FOUND 0)

# nnabla と思われるライブラリとインクルードパスを求める
find_library(libnnabla       NAMES nnabla libnnabla)
find_library(libnnabla_utils NAMES nnabla_utils libnnabla_utils)
find_library(libnnabla_cli   NAMES nnabla_cli libnnabla_cli)

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

if(NNabla_NNabla_FOUND AND NOT TARGET nnabla::nnabla)
  add_library(nnabla::nnabla UNKNOWN IMPORTED)
  set_target_properties(nnabla::nnabla PROPERTIES
    IMPORTED_LOCATION "${NNabla_NNabla_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${NNabla_INCLUDE_DIR}")
endif()

if(NNabla_Utils_FOUND AND NOT TARGET nnabla::nnabla_utils)
  add_library(nnabla::nnabla_utils UNKNOWN IMPORTED)
  set_target_properties(nnabla::nnabla_utils PROPERTIES
    IMPORTED_LOCATION "${NNabla_Utils_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${NNabla_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES nnabla::nnabla)
endif()

if(NNabla_CLI_FOUND AND NOT TARGET nnabla::nnabla_cli)
  add_library(nnabla::nnabla_cli UNKNOWN IMPORTED)
  set_target_properties(nnabla::nnabla_cli PROPERTIES
    IMPORTED_LOCATION "${NNabla_CLI_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${NNabla_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "nnabla::nnabla;nnabla::nnabla_utils")
endif()
