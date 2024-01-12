cmake_minimum_required(VERSION 3.10)
project(colib LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

# Find Pinocchio
# Pinocchio uses its own FindCppAD, but does not provide it.
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
find_package(pinocchio REQUIRED)
include_directories(${PINOCCHIO_INCLUDE_DIRS})

# Find hpp-fcl
find_package(hpp-fcl REQUIRED)
include_directories(${HPP_FCL_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")
include_directories("include")

# Add your project's source files and target
add_executable(test src/articulated_model.cpp)
target_link_libraries(test ${PINOCCHIO_LIBRARIES} ${HPP_FCL_LIBRARIES})
