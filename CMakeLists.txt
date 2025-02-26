cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD 17)
project(sensor_degradation_filter)

# Check whether to build for real Khepera IV experiments
if(NOT DEFINED BUILD_REAL_KHEPERAIV_EXPERIMENT_LOOP_FUNCTIONS)
    set(BUILD_REAL_KHEPERAIV_EXPERIMENT_LOOP_FUNCTIONS FALSE)
endif()

# Find dependencies
find_package(ARGoS REQUIRED)
find_package(GSL REQUIRED)
find_package(NLopt REQUIRED)

# Add ARGoS and Buzz information
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIR}) # include the */lib/argos3 directory to find the ARGoS libraries
link_libraries(${ARGOS_LDFLAGS})

add_subdirectory(src) # add source directory

if(BUILD_REAL_KHEPERAIV_EXPERIMENT_LOOP_FUNCTIONS)
    add_subdirectory(vicon_sdk)
endif()