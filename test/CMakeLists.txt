file(GLOB_RECURSE SOURCES ./*.cc)

foreach(TEST_SOURCE_FILE ${SOURCES})
  file(READ ${TEST_SOURCE_FILE} TEST_SOURCE_CODE)

  # Read test module name
  if(TEST_SOURCE_CODE MATCHES [[BOOST_TEST_MODULE ([a-zA-Z0-9_]+)]])
    # Set executable name to test module name
    set(TEST_EXECUTABLE_NAME ${CMAKE_MATCH_1})
    add_executable(${TEST_EXECUTABLE_NAME} ${TEST_SOURCE_FILE})
    target_link_libraries(${TEST_EXECUTABLE_NAME}
      ai-server-lib
      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
    )
    add_test(${TEST_EXECUTABLE_NAME} ${EXECUTABLE_OUTPUT_PATH}/${TEST_EXECUTABLE_NAME})
  else()
    message(SEND_ERROR "${TEST_SOURCE_FILE}: Test module name is not set!")
  endif()
endforeach()
