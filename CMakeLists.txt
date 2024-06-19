cmake_minimum_required(VERSION 3.16)
project(poly_traj)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)



# Find yaml-cpp package
find_package(yaml-cpp REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package (glog 0.6.0 REQUIRED)

# Create a library from the source files in the src/ directory
add_library(${PROJECT_NAME} STATIC
  src/motion_defines.cpp
  src/polynomial.cpp
  src/segment.cpp
  src/timing.cpp
  src/trajectory.cpp
  src/trajectory_sampling.cpp
  src/vertex.cpp
  src/io.cpp
  src/rpoly/rpoly_ak1.cpp
  src/trajectory_generator.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME} 
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link libraries
target_link_libraries(${PROJECT_NAME} 
  PRIVATE 
    yaml-cpp
    glog::glog
    Eigen3::Eigen
)
