# gtkmm3 が必要
find_package(PkgConfig REQUIRED)
pkg_check_modules(Gtkmm3 REQUIRED IMPORTED_TARGET gtkmm-3.0)

add_executable(standalone-gui main.cc)

target_link_libraries(standalone-gui
  ai-server-common-flags
  ai-server-lib
  PkgConfig::Gtkmm3
)

if(ENABLE_NNABLA_EXT_CUDA)
  target_link_libraries(standalone-gui nnabla::nnabla_cuda)
  # nnabla-ext-cuda が使えるときは AI_SERVER_HAS_NNABLA_EXT_CUDA を define する
  target_compile_definitions(standalone-gui PRIVATE AI_SERVER_HAS_NNABLA_EXT_CUDA)
endif()

ai_server_create_symlink(standalone-gui)

# 設定ファイルのディレクトリを求めやすくするため
# 実行ファイルと同じディレクトリに config への symlink を作る
add_custom_command(
  TARGET standalone-gui
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E create_symlink "${PROJECT_SOURCE_DIR}/config" config
  BYPRODUCTS config
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)
