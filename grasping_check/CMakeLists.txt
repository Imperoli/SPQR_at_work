project(grasping_check)

cmake_minimum_required (VERSION 2.8) 
cmake_policy(SET CMP0015 NEW)

# Change to "Debug" to enable debug symbols
set(CMAKE_BUILD_TYPE "Release")

set(STD_FLAGS_DEBUG "-Wall -g -D__STDC_CONSTANT_MACROS -pthread")
set(STD_FLAGS_RELEASE "-Wall -D__STDC_CONSTANT_MACROS -pthread -export-dynamic -march=nocona -mfpmath=sse -ftree-vectorize -fextended-identifiers -O3")

find_package( OpenCV REQUIRED )
include_directories(. ${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})

#Add here your source files
set(grasping_check_SRCS src/main.cpp src/grasping_check.cpp)

add_executable(grasping_check ${grasping_check_SRCS})
target_link_libraries(grasping_check ${OpenCV_LIBS})
set_target_properties( grasping_check PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
