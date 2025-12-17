cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(Semester_project)

file(GLOB SRC_FILES "src/*.c")
target_sources(app PRIVATE ${SRC_FILES})
target_include_directories(app PRIVATE include)