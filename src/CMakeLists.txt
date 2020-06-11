###### Project Environment #####

# Set minimum CMake version
cmake_minimum_required(VERSION 3.1)

##### Project Setup #####
# Set our output target
set(LIB msckf_vio)

# Source files
file(GLOB SOURCES "*.cpp")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_library(${LIB} STATIC ${SOURCES})
target_include_directories(${LIB} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${SUITESPARSE_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
)

target_link_libraries(${LIB}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    ${SUITESPARSE_LIBRARIES}
    ${Pangolin_LIBRARIES}
    Eigen3::Eigen glog
)
