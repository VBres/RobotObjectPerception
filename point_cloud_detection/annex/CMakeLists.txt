cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(annex)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pclVisualizer pclVisualizer.cpp)
target_link_libraries (pclVisualizer ${PCL_LIBRARIES})
add_executable (pclCreateShape pclCreateShape.cpp)
target_link_libraries (pclCreateShape ${PCL_LIBRARIES})
add_executable (pclIcp pclIcp.cpp)
target_link_libraries (pclIcp ${PCL_LIBRARIES})
