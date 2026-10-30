cmake_minimum_required(VERSION 3.11 FATAL_ERROR)

project(trash-controller CXX)

# Place binaries and libraries according to GNU standards
include(GNUInstallDirs)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

# Include Hardware libraries
include_directories(
    ${PROJECT_SOURCE_DIR}/libs/PCA9685/src 
    ${PROJECT_SOURCE_DIR}/libs/PCA9685/src/build
) 

find_package(Threads)
find_package(cppzmq)
find_package(Curses REQUIRED)

# Test script for point generation
add_executable(
    point_generator
    src/point_generator.cpp
)
target_link_libraries(
    point_generator
    PRIVATE cppzmq ${CMAKE_THREAD_LIBS_INIT}
)
target_include_directories(
    point_generator 
    PUBLIC ${PROJECT_SOURCE_DIR}/include
) 

# Script for command reception
add_executable(
    cmd_reciever
    src/cmd_reciever.cpp
    src/motor.cpp
)
target_link_libraries(
    cmd_reciever 
    PRIVATE cppzmq 
            ${CMAKE_THREAD_LIBS_INIT} 
            ${PROJECT_SOURCE_DIR}/libs/PCA9685/src/build/libPCA9685.so
)
target_include_directories(
    cmd_reciever 
    PUBLIC ${PROJECT_SOURCE_DIR}/include
           ${PROJECT_SOURCE_DIR}/libs/PCA9685/src 
           ${PROJECT_SOURCE_DIR}/libs/I2C/src
) 

# Script for PC controller functionality
add_executable(
    auto_driver
    src/auto_driver.cpp
)
target_link_libraries(
    auto_driver
    PRIVATE cppzmq ${CMAKE_THREAD_LIBS_INIT}
)
target_include_directories(
    auto_driver 
    PUBLIC ${PROJECT_SOURCE_DIR}/include
) 

# Script for manual control
add_executable(
    manual_driver
    src/manual_driver.cpp
    src/motor.cpp
)
target_link_libraries(
    manual_driver 
    PRIVATE cppzmq 
            ${CMAKE_THREAD_LIBS_INIT} 
            ${PROJECT_SOURCE_DIR}/libs/PCA9685/src/build/libPCA9685.so
            ${CURSES_LIBRARIES}
)
target_include_directories(
    manual_driver 
    PUBLIC ${PROJECT_SOURCE_DIR}/include
           ${PROJECT_SOURCE_DIR}/libs/PCA9685/src 
           ${PROJECT_SOURCE_DIR}/libs/I2C/src
) 