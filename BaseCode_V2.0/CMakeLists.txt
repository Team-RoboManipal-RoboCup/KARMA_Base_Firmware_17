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
set(CMAKE_PROJECT_NAME BaseCode_V2.0)

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
    Core/Src/bno085_driver.c
    Core/Src/bno_port_stm32.c

    # Hardware Drivers (Level 2)
    drivers/i2c_recovery.c
    drivers/hw_encoders.c
    drivers/motor_driver.c
    drivers/optical_flow_tof.c
    drivers/bno085.c

    # State Machines, Pure Math, & Logging (Level 2)
    state_machines/imu_sm.c
    Core/Src/algorithm.c
    Core/Src/logger.c

    # FreeRTOS Tasks & Wiring (Level 3 & 4)
    tasks/motor_task.c
    tasks/acquisition_task.c
    tasks/estimation_task.c
    tasks/communication_task.c
    tasks/task_manager.c

    # FreeRTOS Kernel Sources (Strict Zero-Heap)
    ThirdParty/FreeRTOS/Source/tasks.c
    ThirdParty/FreeRTOS/Source/queue.c
    ThirdParty/FreeRTOS/Source/list.c
    ThirdParty/FreeRTOS/Source/timers.c
    ThirdParty/FreeRTOS/Source/event_groups.c
    ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Core/Inc
    .
    drivers
    state_machines
    tasks
    ThirdParty/FreeRTOS/Source/include
    ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
