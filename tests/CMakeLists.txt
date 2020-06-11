cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

file(GLOB SOURCES
    "functional_test/*.cpp"
    "functional_test/tum/*.cpp"
    "functional_test/kitti/*.cpp"
    "functional_test/utils/*.cpp"
    "functional_test/ocams/*.cpp"
)
# message(STATUS ${SOURCES})

include_directories(${CMAKE_CURRENT_SOURCE_DIR/functional_test})
include_directories(${CMAKE_CURRENT_SOURCE_DIR/functional_test/tum})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/functional_test/kitti)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/functional_test/ocams)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/utils)

add_executable(test_vio samples/test_vio.cpp ${SOURCES})
target_include_directories(test_vio PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/
    ${CMAKE_CURRENT_SOURCE_DIR}/functional_test/
)
target_link_libraries(test_vio 
    msckf_vio
    pthread
    stdc++fs
)
