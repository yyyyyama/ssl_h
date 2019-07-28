# 対象をビルドするかをコマンドラインオプションから指定できる add_subdirectory() っぽいもの
# cmake コマンド実行時に -DWITH_name=ON/OFF を渡すことで有効/無効を切り替えることができる
# @param name サブディレクトリ名
# @param buid -DWITH_name=ON/OFF の初期値
function(ai_server_add_subdirectory name build)
  set(option_name "WITH_${name}")
  option(${option_name} "Build ${name}" ${build})
  set(option_value "${${option_name}}")

  if (option_value)
    add_subdirectory(${name})
  endif()
endfunction()

# target への symlink をビルドディレクトリ直下に作成する
function(ai_server_create_symlink target)
  get_filename_component(target_abspath ${target} ABSOLUTE BASE_DIR ${CMAKE_CURRENT_BINARY_DIR})
  file(RELATIVE_PATH target_relpath ${CMAKE_BINARY_DIR} ${target_abspath})
  get_filename_component(link_abspath ${target} ABSOLUTE BASE_DIR ${CMAKE_BINARY_DIR})
  add_custom_command(
    TARGET ${target}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${target_relpath} ${target}
    BYPRODUCTS ${link_abspath}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
endfunction()

ai_server_add_subdirectory(ai-server ON)
ai_server_add_subdirectory(standalone-gui ON)
