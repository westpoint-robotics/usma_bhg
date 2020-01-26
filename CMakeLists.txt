cmake_minimum_required(VERSION 2.8.3)
project(usma_bhg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
#find_library(PNG16 NAMES png16 libpng16 lpng16 PATHS /usr/lib/x86_64-linux-gnu /snap/core/7270/usr/lib/x86_64-linux-gnu)
#if(NOT PNG12)
#  message(FATAL_ERROR "PNG16 library not found")
#else()
#  message(FATAL_ERROR "PNG16 library found")
#    
#endif()

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
target_link_libraries(gobi_subscriber ${catkin_LIBRARIES} -lxeneth -lstdc++ )
#add_dependencies(bool_pub beginner_tutorials_generate_messages_cpp)

#add_executable(bool_sub src/bool_sub.cpp)
#target_link_libraries(bool_sub ${catkin_LIBRARIES} -lSpinnaker -lswscale -lswresample -lavcodec -lavutil -lavformat)

#add_dependencies(bool_sub beginner_tutorials_generate_messages_cpp)

#add_executable(camera_subscriber src/camera_subscriber.cpp)
#target_link_libraries(camera_subscriber ${catkin_LIBRARIES} -lSpinnaker -lswscale -lswresample -lavcodec -lavutil -lavformat)


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/usma_bhg.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/usma_bhg_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

