cmake_minimum_required(VERSION 3.6)
project(dynim-dns-vio-algo)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(${CMAKE_SOURCE_DIR}/cmake/pkg_check_modules_wrapper.cmake)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})

# try using c++17, if c++17 does not find, a previous standard will be used
set(CMAKE_CXX_STANDARD 17)

# Find packages
find_package( OpenCV REQUIRED )
find_package( glog REQUIRED )

# Create a new interface of eigen3 when cross-compile
# otherwise, use default cmake/FindSuiteSparse.cmake
if(CMAKE_SYSROOT)
    set(Eigen3_INCLUDE_DIRS ${CMAKE_SYSROOT}/usr/include/eigen3)

    # Create imported target Eigen3::Eigen
    add_library(Eigen3::Eigen INTERFACE IMPORTED)

    set_target_properties(Eigen3::Eigen PROPERTIES
  	INTERFACE_INCLUDE_DIRECTORIES "${Eigen3_INCLUDE_DIRS}"
    )

else()
    find_package( Eigen3 3.3 REQUIRED NO_MODULE )
endif()

find_package( SuiteSparse REQUIRED )
find_package( Boost COMPONENTS system REQUIRED )
find_package( Pangolin REQUIRED )

include_directories("src/")
add_subdirectory(src)
add_subdirectory(tests)

file(GLOB DATA_FILES "config/*.*")
file(COPY ${DATA_FILES} DESTINATION ${PROJECT_BINARY_DIR}/config)
