cmake_minimum_required(VERSION 3.20.0)


set(APP_CONF_FILE ${CMAKE_CURRENT_SOURCE_DIR}/prj.conf)



find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(stm32mini_gps)

target_sources(app PRIVATE src/main.c
        src/ublox_m10_nano.c
        src/lis3dh.c
        src/walk.c
)
