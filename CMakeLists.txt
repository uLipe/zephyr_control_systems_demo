cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zephyr_motor_controller)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/motor_controller.c)
target_sources(app PRIVATE src/adrc_control_law.c)
target_sources(app PRIVATE src/pid_control_law.c)
target_sources(app PRIVATE src/motor_hardware_mf4005.c)
