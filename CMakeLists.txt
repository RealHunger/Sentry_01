cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME sentry_gimbal)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Application/struct_typedef.h
        Bsp/can/bsp_can.c
        Bsp/can/bsp_can.h
        Bsp/LED/bsp_LED.c
        Bsp/LED/bsp_LED.h
        Bsp/uart/bsp_uart.c
        Bsp/uart/bsp_uart.h
        Components/remote/remote.c
        Components/remote/remote.h
        Components/ist8310_i2c/ist8310_driver.c
        Components/ist8310_i2c/ist8310_driver.h
        Components/BMI088/BMI088driver.c
        Components/BMI088/BMI088Middleware.c
        Algorithm/MahonyAHRS/MahonyAHRS.c
        Application/mahony.c
        Application/mahony.h
        Application/auto_ctrl.c
        Application/auto_ctrl.h
        Bsp/usb_cdc/bsp_usb_cdc.c
        Bsp/usb_cdc/bsp_usb_cdc.h
        Components/motor/motor.c
        Components/ist8310_i2c/ist8310_middleware.c
        Components/ist8310_i2c/ist8310_middleware.h
        Application/robot_global.c
        Application/robot_global.h
        ALL_Task/sensor_task.c
        ALL_Task/sensor_task.h
        ALL_Task/motor_task.c
        ALL_Task/motor_task.h
        ALL_Task/chassis_task.c
        ALL_Task/chassis_task.h
        ALL_Task/gimbal_task.c
        ALL_Task/gimbal_task.h
        Application/auto_ctrl.c
        ALL_Task/recv_cmd_task.c
        ALL_Task/recv_cmd_task.h
        ALL_Task/shoot_task.c
        ALL_Task/shoot_task.h
        ALL_Task/send_info_task.c
        ALL_Task/send_info_task.h
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
        -u _printf_float
)

target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx
        -u _printf_float
)

