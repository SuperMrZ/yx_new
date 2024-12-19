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



# 设置 ARM DSP 库的路径
set(ARM_DSP_LIB ${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib/libarm_cortexM4lf_math.a)


# 检查库文件是否存在
if(NOT EXISTS ${ARM_DSP_LIB})
    message(FATAL_ERROR "ARM DSP library not found at: ${ARM_DSP_LIB}")
endif()

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME yx_new_f407)

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
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)



# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    BSP/Src/bsp_can.c
    BSP/Src/bsp_uart.c
    Alogrithm/Src/PID.c
    Device/Src/motorCmd.c
    Device/Src/damiao.c
    Tasks/Src/tasks.c
    Alogrithm/Src/controller.c
    Alogrithm/Src/kalman_filter.c
    Alogrithm/Src/QuaternionEKF.c
    Alogrithm/Src/transfer.c
    Alogrithm/Src/user_lib.c
    Alogrithm/Src/crc.c

    BSP/Src/bsp_dwt.c
    BSP/Src/bsp_PWM.c
    BSP/Src/bsp_rc.c
 
    Device/Src/BMI088driver.c
    Device/Src/BMI088Middleware.c
    Tasks/Src/ins_task.c


    


    
 





)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    BSP/Inc
    Alogrithm/Inc
    Device/Inc
    Device/Inc
    Tasks/Inc

    


)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    ${ARM_DSP_LIB}

    # Add user defined libraries
)

