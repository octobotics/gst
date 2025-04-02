cmake_minimum_required(VERSION 3.0.2)
project(gst)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)

include_directories(${GSTREAMER_INCLUDE_DIRS})
link_directories(${GSTREAMER_LIBRARY_DIRS})
add_definitions(${GSTREAMER_CFLAGS_OTHER})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  message_generation
  std_srvs
)

find_package(OpenCV REQUIRED)


## Add service files
add_service_files(
  FILES
  CaptureFrame.srv
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime std_srvs
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(gstreamer_node src/main.cpp)
#target_link_libraries(gstreamer_node ${catkin_LIBRARIES} ${GSTREAMER_LIBRARIES})


## Specify libraries to link a library or executable target against
target_link_libraries(gstreamer_node
  ${catkin_LIBRARIES}
  gstapp-1.0
  gstreamer-1.0
  gstbase-1.0
  gobject-2.0
  glib-2.0
  ${GSTREAMER_LIBRARIES}
)

## Add dependencies to the executable
add_dependencies(gstreamer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
