cmake_minimum_required(VERSION 2.8.3)
project(mrasl_guidance)

include(CheckCXXCompilerFlag)
#set(CMAKE_BUILD_TYPE Debug)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  std_msgs
  sensor_msgs
  image_transport
  geometry_msgs
  dynamic_reconfigure
  camera_info_manager
  tf2
  tf2_ros
  message_generation
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/guidance.cfg
)

add_service_files(
  FILES
  toggle_camera.srv
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  roscpp
                  cv_bridge
                  std_msgs
                  sensor_msgs
                  image_transport
                  geometry_msgs
                  dynamic_reconfigure
                  camera_info_manager
                  tf2
                  tf2_ros
                  message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

MESSAGE("System is: " ${CMAKE_SYSTEM_PROCESSOR})
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  if (CMAKE_SIZEOF_VOID_P EQUAL 4)
    MESSAGE("-- 32bit detected")
    link_directories(${PROJECT_SOURCE_DIR}/lib/x86)
  elseif (CMAKE_SIZEOF_VOID_P EQUAL 8)
    MESSAGE("-- 64bit detected")
    link_directories(${PROJECT_SOURCE_DIR}/lib/x64)
  endif ()
else()
  MESSAGE("-- Non-linux platform detected but sorry we do not support :D")
endif ()

if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "armv7l" )
  MESSAGE("-- " ${CMAKE_SYSTEM_PROCESSOR} " detected")
  link_directories(${PROJECT_SOURCE_DIR}/lib/XU3)
else ()
endif ()

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

add_executable( guidance
  ${PROJECT_SOURCE_DIR}/src/guidance_node.cpp
  ${PROJECT_SOURCE_DIR}/src/guidance_manager.cpp
  ${PROJECT_SOURCE_DIR}/src/guidance_configuration.cpp
  ${PROJECT_SOURCE_DIR}/src/dji/utils.cpp
)

add_dependencies(guidance
  ${PROJECT_NAME}_gencfg
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries( guidance
  DJI_guidance
  usb-1.0
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
