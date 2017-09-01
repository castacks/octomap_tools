cmake_minimum_required(VERSION 2.8.3)
project(octomap_tools)

# -----------------------------------------------
# USER DEFINED VARIABLES
# -----------------------------------------------
option(BUILD_EXECUTABLE "Build as executable to test functionality"                     ON)


find_package(catkin REQUIRED COMPONENTS 
roscpp pcl_conversions pcl_ros message_filters sensor_msgs cmake_modules  octomap_msgs)
find_package(Boost REQUIRED COMPONENTS random)
find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

# find_package(dynamicEDT3D)
# if (${dynamicEDT3D_FOUND})                                                                       
#   include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})
#   MESSAGE(WARNING ${DYNAMICEDT3D_INCLUDE_DIRS})                                                         
#   link_directories(${DYNAMICEDT3D_LIBRARY_DIRS})
#   list(APPEND LIB_FILES src/EndpointModel.cpp)                                      
# else (${dynamicEDT3D_FOUND})
#   MESSAGE(WARNING "dynamicEDT3D library (part of OctoMap >1.5) not found, skipping endpoint model")
#   add_definitions(-DSKIP_ENDPOINT_MODEL)
# endif (${dynamicEDT3D_FOUND})


# -----------------------------------------------
# COMPILER FLAGS
# -----------------------------------------------

# set the build type: "Choose the type of build. Options are: None, Debug, Release, RelWithDebInfo, MinSizeRel"
set(CMAKE_BUILD_TYPE Release)


# add vectorization support
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4.1")
# set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -msse4.1")

# use highest level of optimization in Release mode
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -O3")
endif ()


catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS tf
  DEPENDS octomap OpenMP
)

include_directories(#include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cloud_transform src/cloud_transform.cpp)
target_link_libraries(cloud_transform ${catkin_LIBRARIES})

add_executable(cloud_passthrough src/cloud_passthrough.cpp)
target_link_libraries(cloud_passthrough ${catkin_LIBRARIES})

add_executable(cloud2octomap src/cloud2octomap.cpp)
target_link_libraries(cloud2octomap ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(clouds2octomap src/clouds2octomap.cpp)
target_link_libraries(clouds2octomap ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(cloud_downsample src/cloud_downsample.cpp)
target_link_libraries(cloud_downsample ${catkin_LIBRARIES} )


# -----------------------------------------------
# BUILD SUMARY
# -----------------------------------------------
message("")
message("---------------------------------------------")
message(${PROJECT_NAME} " Configuration Summary")
message("")
message("Build type:                    ${CMAKE_BUILD_TYPE}")
message("Build with threading (OpenMP): ${WITH_OPENMP}")
message("Build as executable:           ${BUILD_EXECUTABLE}")
message("---------------------------------------------")
message("")
