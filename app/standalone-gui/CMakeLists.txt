# gtkmm3 が必要
find_package(PkgConfig REQUIRED)
pkg_check_modules(Gtkmm3 REQUIRED gtkmm-3.0)

add_executable(standalone-gui main.cc)

# gtkmm 関連のヘッダファイルに出てくる
#   warning: named variadic macros are a GNU extension [-Wvariadic-macros]
# を静かにさせる
set_target_properties(standalone-gui PROPERTIES
  COMPILE_FLAGS "-Wno-variadic-macros"
)

# standalone-gui ターゲットに追加で必要な インクルードパスやライブラリを指定する
# 全体で必要なライブラリ (Boost とか) は #298 の変更で必要なくなった
# (target_link_libraries に ai-server-lib を指定すると依存ライブラリの設定も行われるようにした)
target_include_directories(standalone-gui PRIVATE
  ${Gtkmm3_INCLUDE_DIRS}
)
target_link_libraries(standalone-gui
  ai-server-lib
  ${Gtkmm3_LIBRARIES}
)

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
