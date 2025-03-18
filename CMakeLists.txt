cmake_minimum_required(VERSION 3.1)
project(cameras_pose_optimization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# Find required packages
find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt5)
find_package(yaml-cpp REQUIRED)

# Enable CGAL viewer
if(CGAL_Qt5_FOUND)
    add_definitions(-DCGAL_USE_BASIC_VIEWER -DCGAL_USE_QT5)
    message(STATUS "Found CGAL Qt5")
else()
    message(FATAL_ERROR "CGAL Qt5 component is required but not found!")
endif()

# Add include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CGAL_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

# Add executable
add_executable(cameras_optimizer 
    src/main.cpp
    src/camera.cpp
    src/robot.cpp
    src/config_reader.cpp
)

# Link libraries
target_link_libraries(cameras_optimizer
    CGAL::CGAL
    CGAL::CGAL_Qt5
    yaml-cpp
) 