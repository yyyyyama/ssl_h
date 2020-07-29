file(GLOB_RECURSE SOURCES ./*.cc)

foreach(TEST_SOURCE_FILE ${SOURCES})
  file(RELATIVE_PATH SRC_RELPATH ${CMAKE_CURRENT_LIST_DIR} ${TEST_SOURCE_FILE})
  string(REGEX REPLACE "\.cc$" "" TEST_MODULE_NAME "test/${SRC_RELPATH}")
  string(REPLACE "/" "_" TEST_EXECUTABLE_NAME ${TEST_MODULE_NAME})

  add_executable(${TEST_EXECUTABLE_NAME} ${TEST_SOURCE_FILE})
  target_compile_definitions(${TEST_EXECUTABLE_NAME} PRIVATE BOOST_TEST_MODULE=${TEST_MODULE_NAME})
  target_include_directories(${TEST_EXECUTABLE_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR})
  target_link_libraries(${TEST_EXECUTABLE_NAME}
    ai-server-common-flags
    ai-server-lib
    Boost::unit_test_framework

    # radio::connention::serial のテストで使う openpty に必要
    # TODO: 特定のテストで必要なライブラリを指定できるようにして serial のテストのみにリンクさせる
    # TODO: Linux 環境以外に対応する...？
    util
  )
  add_test(${TEST_MODULE_NAME} ${TEST_EXECUTABLE_NAME})
endforeach()
