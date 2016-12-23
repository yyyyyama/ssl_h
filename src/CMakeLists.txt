add_subdirectory(ai_server)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})
add_executable(${PROJECT_NAME} main.cc)
target_link_libraries(${PROJECT_NAME} ai-server-lib)
