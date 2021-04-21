cmake_minimum_required(VERSION 2.8.3)
project(mpc_follower2)

find_package(
  catkin REQUIRED COMPONENTS
  amathutils_lib
  autoware_build_flags
  autoware_health_checker
  autoware_msgs
  geometry_msgs
  qpoases_vendor
  roscpp
  rostest
  rosunit
  sensor_msgs
  std_msgs
  tf
  tf2
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    amathutils_lib
    autoware_health_checker
    autoware_msgs
    geometry_msgs
    qpoases_vendor
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf2
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${qpoases_vendor_INCLUDE_DIRS}/qpoases_vendor
)

set(
  MPC_FOLLOWER_SRC
    src/mpc_utils.cpp
    src/mpc_trajectory.cpp
    src/lowpass_filter.cpp
    src/vehicle_model/vehicle_model_interface.cpp
    src/vehicle_model/vehicle_model_bicycle_kinematics.cpp
    src/vehicle_model/vehicle_model_bicycle_dynamics.cpp
    src/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.cpp
    src/qp_solver/qp_solver_unconstr.cpp
    src/qp_solver/qp_solver_unconstr_fast.cpp
    src/qp_solver/qp_solver_qpoases.cpp
)

add_executable(mpc_follower2 src/mpc_follower_node.cpp src/mpc_follower_core.cpp ${MPC_FOLLOWER_SRC})
add_dependencies(mpc_follower2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_follower2 ${catkin_LIBRARIES})

add_executable(mpc_waypoints_converter2 src/mpc_waypoints_converter.cpp)
add_dependencies(mpc_waypoints_converter2 ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_waypoints_converter2 ${catkin_LIBRARIES})

install(
  TARGETS mpc_follower2 mpc_waypoints_converter2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    add_rostest_gtest(
      test-mpc_utils2
      test/test_mpc_utils.test
      test/src/test_mpc_utils.cpp
      ${MPC_FOLLOWER_SRC}
    )
    add_dependencies(test-mpc_utils2 ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test-mpc_utils2 ${catkin_LIBRARIES})

    add_rostest_gtest(
      test-mpc_follower2
      test/test_mpc_follower.test
      test/src/test_mpc_follower.cpp
      src/mpc_follower_core.cpp
      ${MPC_FOLLOWER_SRC}
    )
    add_dependencies(test-mpc_follower2 ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test-mpc_follower2 ${catkin_LIBRARIES})

    add_rostest_gtest(
      test-mpc_lowpass_filter2
      test/test_mpc_lowpass_filter.test
      test/src/test_mpc_lowpass_filter.cpp
      src/lowpass_filter.cpp
    )
    add_dependencies(test-mpc_lowpass_filter2 ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test-mpc_lowpass_filter2 ${catkin_LIBRARIES})
endif()
