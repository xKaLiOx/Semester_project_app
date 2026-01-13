cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(Semester_project)

target_include_directories(app PRIVATE
    src
    stm_drivers/wrapper
    stm_drivers/st1vafe3bx-pid
    stm_drivers/st1vafe6ax-pid
)


#add_subdirectory()
target_sources(app PRIVATE
    src/main.c
    stm_drivers/wrapper/st1vafe_wrapper.c
    stm_drivers/st1vafe3bx-pid/st1vafe3bx_reg.c
    stm_drivers/st1vafe6ax-pid/st1vafe6ax_reg.c
)