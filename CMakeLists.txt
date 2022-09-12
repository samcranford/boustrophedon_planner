cmake_minimum_required(VERSION 3.1...3.24)

project(
  BoustrophedonPlanner
  VERSION 1.0
  LANGUAGES CXX)

add_library(BoustrophedonLibrary STATIC src/boustrophedon_algorithms.cpp)
target_include_directories(BoustrophedonLibrary PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(BoustrophedonExecutable src/main.cpp)

target_link_libraries(BoustrophedonExecutable PRIVATE BoustrophedonLibrary)