add_executable(example-test example.cc)
target_link_libraries(example-test ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(example-test ${EXECUTABLE_OUTPUT_PATH}/example-test)
