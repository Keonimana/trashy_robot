cmake_minimum_required(VERSION 3.11 FATAL_ERROR)

project(trash-camera CXX)

# place binaries and libraries according to GNU standards
include(GNUInstallDirs)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

add_custom_target(cleanup
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/bin
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/CMakeFiles
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/cmake_install.cmake
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/Makefile
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/CMakeCache.txt
    COMMENT "Cleaning build artifacts"
)

find_package(Threads)
find_package(cppzmq)
find_package(depthai CONFIG REQUIRED)

# OAK-D Lite RGB and depth publisher
add_executable(
    camera
    src/camera.cpp
)
target_link_libraries(
    camera
    PRIVATE cppzmq ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} depthai::opencv
)
target_include_directories(
    camera 
    PUBLIC ${PROJECT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../libs/depthai-core/shared/depthai-shared/include
) 