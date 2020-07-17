add_executable(ai-server main.cc)
target_link_libraries(ai-server ai-server-common-flags ai-server-lib)
ai_server_create_symlink(ai-server)
