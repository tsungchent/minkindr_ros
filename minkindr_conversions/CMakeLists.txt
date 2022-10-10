cmake_minimum_required(VERSION 3.15)
project(minkindr_conversions)

find_package(ament_cmake REQUIRED)

# add_definitions(-std=c++11)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 11)


find_package(eigen_vendor REQUIRED)
# find_package(minkindr REQUIRED)
find_package(glog_vendor REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(${CMAKE_INSTALL_PREFIX}/include)

##########
# GTESTS #
##########
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(kindr_tf_test
  test/kindr_tf_test.cc
)
# target_link_libraries(kindr_tf_test ${catkin_LIBRARIES})
ament_target_dependencies(kindr_tf_test
  eigen_vendor
  glog_vendor
  tf2
  tf2_eigen
  geometry_msgs
)

target_include_directories(
  kindr_tf_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>
)

ament_add_gtest(kindr_msg_test
  test/kindr_msg_test.cc
)
ament_target_dependencies(kindr_msg_test
  eigen_vendor
  glog_vendor
  tf2
  tf2_eigen
  geometry_msgs
)

target_include_directories(
  kindr_msg_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>
)

# target_link_libraries(kindr_msg_test ${catkin_LIBRARIES})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
   DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

##########
# EXPORT #
##########
ament_package()
