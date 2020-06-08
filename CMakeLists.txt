cmake_minimum_required(VERSION 2.8.3)
project(usma_bhg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
SET(CMAKE_CXX_FLAGS "-std=c++11 -O3")

find_package(OpenCV REQUIRED)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
image_transport
)

#find_library(PNG12 NAMES png12 libpng12 libpng16 PATHS /snap/core/7270/lib/x86_64-linux-gnu /snap/core/7270/usr/lib/x86_64-linux-gnu)
#if(NOT PNG12)
#  message(FATAL_ERROR "PNG12 library not found")
#endif()
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES usma_bhg
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include 
    ${catkin_INCLUDE_DIRS}
    /usr/share/xeneth/Include
#    /snap/core/7270/lib/x86_64-linux-gnu
#    /usr/include/spinnaker    
    )

#link_directories(/snap/core/7270/lib/x86_64-linux-gnu)



#add_executable(bool_pub src/bool_pub.cpp)
add_executable(gobi_subscriber src/gobi_subscriber.cpp)
target_link_libraries(gobi_subscriber ${catkin_LIBRARIES} -lxeneth -lstdc++ -lusb-1.0  ${OpenCV_LIBS})


add_executable(gobi_oop src/gobi_subscriber_oop.cpp)
target_link_libraries(gobi_oop ${catkin_LIBRARIES} -lxeneth -lstdc++ -lusb-1.0  ${OpenCV_LIBS})


add_executable(gobi_trig_in src/gobi_trig.cpp)
target_link_libraries(gobi_trig_in ${catkin_LIBRARIES} -lxeneth -lstdc++ -lusb-1.0  ${OpenCV_LIBS})
