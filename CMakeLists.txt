cmake_minimum_required(VERSION 3.5)

project(ai-server CXX)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

# Disable in-source build
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES  ON)

# Check operating system
if(NOT (CMAKE_SYSTEM_NAME STREQUAL "Linux" OR CMAKE_SYSTEM_NAME STREQUAL "Darwin"))
  message(FATAL_ERROR "Currently, your platform ${CMAKE_SYSTEM_NAME} is not supported...")
endif()

# Set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Check if the compiler supports C++14
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG(-std=c++14 COMPILER_SUPPORTS_CXX14)
if(NOT COMPILER_SUPPORTS_CXX14)
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support!")
endif()

# Set compiler flags
set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS}         -std=c++14 -Wall -Wextra -pedantic")
set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   -g -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -march=native -DNDEBUG")
set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS}  -Wl,-O1,--sort-common,--as-needed,-z,relro")

if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  if (CMAKE_GENERATOR STREQUAL "Ninja")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fcolor-diagnostics")
  endif()
endif()

# Sanitizers
if (ENABLE_SANITIZERS)
  if(CMAKE_COMPILER_IS_GNUCXX)
    message(STATUS "Enabling GCC's address sanitizer")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    message(STATUS "Enabling Clang's address sanitizer")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
  endif()
endif()

# Code coverage
if(ENABLE_CODE_COVERAGE)
  find_program(GCOV_PATH gcov)
  if(GCOV_PATH)
    message(STATUS "Found gcov: ${GCOV_PATH}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
  endif()
endif()

# Required libraries
find_package(Boost 1.59.0 COMPONENTS coroutine system unit_test_framework REQUIRED)
find_package(Eigen3 3.2 REQUIRED)
find_package(Protobuf 2.6 REQUIRED)
find_package(Threads REQUIRED)

# Enable CTest
enable_testing()

# Include directories
include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${Boost_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
)

if(EIGEN3_VERSION_STRING VERSION_LESS 3.2.0)
  include_directories(${EIGEN3_INCLUDE_DIR})
else()
  include_directories(${EIGEN3_INCLUDE_DIRS})
endif()

add_subdirectory(3rd)
add_subdirectory(src)
add_subdirectory(test)
