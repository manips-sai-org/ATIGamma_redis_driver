cmake_minimum_required (VERSION 2.8)
project (ATIGamma_redis_driver)

# CMAKE OPTIONS
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE Release)

set(SOURCES
    ATIGamma_redis_driver.cpp
)

# external libraries
# - hiredis
find_library(HIREDIS_LIBRARY hiredis)

# - eigen3
find_package(Eigen3 REQUIRED)

# jsoncpp
if (CMAKE_SYSTEM_NAME MATCHES Darwin) 
  find_library(JSONCPP_LIBRARY jsoncpp)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(JSONCPP jsoncpp)
  link_directories(${JSONCPP_LIBRARY_DIRS})
  set(JSONCPP_LIBRARY ${JSONCPP_LIBRARIES})
endif ()

SET(ATIGAMMA_DRIVER_INCLUDE_DIRS
    ${EIGEN3_INCLUDE_DIR}
    ${JSONCPP_INCLUDE_DIRS}  
    )

# Create a library
include_directories(${ATIGAMMA_DRIVER_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${HIREDIS_LIBRARY})
