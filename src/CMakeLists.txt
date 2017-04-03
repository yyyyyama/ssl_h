add_subdirectory(ai_server)

include_directories(${CMAKE_BINARY_DIR}/3rd)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})
add_executable(${PROJECT_NAME} main.cc)
target_link_libraries(${PROJECT_NAME}
  ai-server-lib
  ssl-protos
  ${Boost_COROUTINE_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
  ${Protobuf_LIBRARY}
  ${CMAKE_THREAD_LIBS_INIT}
)
