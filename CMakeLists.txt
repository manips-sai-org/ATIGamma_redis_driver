cmake_minimum_required (VERSION 2.8)
project (ATIGamma_redis_driver)

# CMAKE OPTIONS
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_CXX_FLAGS "-std=c++11")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  # Set the default build, and also the possible values of build type for
  # cmake-gui
  set(CMAKE_BUILD_TYPE
      Release
      CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
                                               "MinSizeRel" "RelWithDebInfo")
endif()

set(SOURCES
    ATIGamma_redis_driver.cpp
)

# - tinyxml2
find_library(TINYXML_LIBRARIES tinyxml2)

# - eigen3
find_package(Eigen3 REQUIRED)

# SaiCommon
find_package(SAI-COMMON REQUIRED)

SET(ATIGAMMA_DRIVER_INCLUDE_DIRS
    ${EIGEN3_INCLUDE_DIR}
	${TINYXML_INCLUDE_DIR}
	${SAI-COMMON_INCLUDE_DIRS}
    )

# Create a library
include_directories(${ATIGAMMA_DRIVER_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${SAI-COMMON_LIBRARIES} ${TINYXML_LIBRARIES})

set(CONFIG_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}/config_folder")
add_definitions(-DCONFIG_FOLDER="${CONFIG_FOLDER}")