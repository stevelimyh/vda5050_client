cmake_minimum_required(VERSION 3.8)
project(vda5050_master_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(fmt REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(vda5050_core REQUIRED)
find_package(vda5050_interfaces REQUIRED)

include(GNUInstallDirs)

# --- rosidl interfaces -------------------------------------------------------
set(msg_files
  msg/AgvAlignment.msg
  msg/AgvAlignmentFinding.msg
  msg/AGVKey.msg
  msg/AGVOnboardSpec.msg
  msg/AssignOrderRequest.msg
  msg/AssignmentResult.msg
  msg/DeviceStatus.msg
  msg/FleetRoster.msg
  msg/MasterConnection.msg
  msg/OrderStatus.msg
  msg/PoseView.msg
)
set(srv_files
  srv/AssignInstantActions.srv
  srv/AssignOrder.srv
  srv/DiscardModeCancelledQueue.srv
  srv/GetDeviceStatus.srv
  srv/GetLoadedMap.srv
  srv/GetMasterBrokerStatus.srv
  srv/GetOrderStatus.srv
  srv/GetPoseView.srv
  srv/OffboardAGV.srv
  srv/OffboardAGVBatch.srv
  srv/OnboardAGV.srv
  srv/OnboardAGVBatch.srv
  srv/ResumeModeCancelledQueue.srv
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files} ${srv_files}
  DEPENDENCIES vda5050_interfaces builtin_interfaces
  ADD_LINTER_TESTS
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# --- library -----------------------------------------------------------------
file(GLOB_RECURSE ros2_srcs "src/*.cpp")
add_library(${PROJECT_NAME}_lib SHARED ${ros2_srcs})
target_compile_definitions(${PROJECT_NAME}_lib PUBLIC ENABLE_ROS2)
target_link_libraries(${PROJECT_NAME}_lib PUBLIC
  ${cpp_typesupport_target}
  vda5050_core::master
  vda5050_core::types
  vda5050_core::json_utils
  vda5050_core::execution
  vda5050_core::transport
  vda5050_core::logger
  vda5050_core::errors
  vda5050_core::order_utils
  fmt::fmt
  rclcpp::rclcpp
  ${vda5050_interfaces_TARGETS}
  ${builtin_interfaces_TARGETS}
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
set_target_properties(${PROJECT_NAME}_lib PROPERTIES
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE)

# --- example executable ------------------------------------------------------
add_executable(example_master examples/example_master.cpp)
target_link_libraries(example_master
  ${PROJECT_NAME}_lib
  rclcpp::rclcpp
  ament_index_cpp::ament_index_cpp
)

# --- example_client executable (interop testing) ----------------------------
# AGV-side reference. SECTION 1 is a verbatim port of Saurabh's example_client
# from his throwaway/test-client branch (mechanical changes for the current
# single-package layout). SECTION 2 adds a MinimalStatePublisher — test-only,
# not for production — so the master's pre_send_validator can accept orders.
# Run with:
#   ros2 run vda5050_master_ros2 example_client
add_executable(example_client examples/example_client.cpp)
target_link_libraries(example_client
  vda5050_core::execution
  vda5050_core::transport
  vda5050_core::types
  vda5050_core::json_utils
  vda5050_core::logger
  fmt::fmt
)
set_target_properties(example_client PROPERTIES
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE)

# --- mock_fms executable (FMS-side console for interop testing) -------------
# Drives the master's 10 ROS 2 service endpoints + subscribes to per-AGV
# state/connection/order_status topics. Three modes: scripted scenarios,
# interactive REPL, and live monitor. See examples/mock_fms.cpp header for
# usage. Run with:
#   ros2 run vda5050_master_ros2 mock_fms --interactive
add_executable(mock_fms examples/mock_fms.cpp)
target_link_libraries(mock_fms
  ${cpp_typesupport_target}
  rclcpp::rclcpp
  ${vda5050_interfaces_TARGETS}
  fmt::fmt
)
set_target_properties(mock_fms PROPERTIES
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE)

# --- mock_client executable (standalone AGV stub for manual testing) --------
# Standalone pure-VDA5050-over-MQTT AGV stub. Deliberately does NOT link
# ${PROJECT_NAME}_lib (no rclcpp dependency) or vda5050_core::master (mock is
# the AGV side, not master). Run with:
#   ros2 run vda5050_master_ros2 mock_client --serial MOCK001 ...
add_executable(mock_client examples/mock_client.cpp)
target_link_libraries(mock_client
  vda5050_core::execution
  vda5050_core::transport
  vda5050_core::types
  vda5050_core::json_utils
  vda5050_core::logger
  fmt::fmt
)
set_target_properties(mock_client PROPERTIES
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE)

# --- install -----------------------------------------------------------------
install(DIRECTORY include/${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY sample_data/ DESTINATION share/${PROJECT_NAME}/sample_data)
install(TARGETS ${PROJECT_NAME}_lib example_master mock_client example_client mock_fms
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rosidl_default_runtime vda5050_core vda5050_interfaces)

# --- tests -------------------------------------------------------------------
if(BUILD_TESTING)
  vda5050_ament_lint_common()
  find_package(ament_cmake_gmock REQUIRED)

  set(_next_domain 41)
  macro(_add_ros2_test name src)
    ament_add_gmock(${name} ${src})
    target_link_libraries(${name}
      ${PROJECT_NAME}_lib
      rclcpp::rclcpp
      ${vda5050_interfaces_TARGETS}
    )
    set_target_properties(${name} PROPERTIES
      BUILD_RPATH_USE_ORIGIN TRUE
      INSTALL_RPATH_USE_LINK_PATH TRUE)
    set_tests_properties(${name} PROPERTIES
      ENVIRONMENT "ROS_DOMAIN_ID=${_next_domain};RMW_IMPLEMENTATION=rmw_cyclonedds_cpp")
    math(EXPR _next_domain "${_next_domain} + 1")
  endmacro()

  _add_ros2_test(test_device_status_publisher    test/unit/test_unit_device_status_publisher.cpp)
  _add_ros2_test(test_device_status_service      test/unit/test_unit_device_status_service.cpp)
  _add_ros2_test(test_order_status_publisher     test/unit/test_unit_order_status_publisher.cpp)
  _add_ros2_test(test_order_status_service       test/unit/test_unit_order_status_service.cpp)
  _add_ros2_test(test_pose_view_publisher        test/unit/test_unit_pose_view_publisher.cpp)
  _add_ros2_test(test_order_send_service         test/unit/test_unit_order_send_service.cpp)
  _add_ros2_test(test_instant_actions_send_service test/unit/test_unit_instant_actions_send_service.cpp)
  _add_ros2_test(test_onboard_agv_service        test/unit/test_unit_onboard_agv_service.cpp)
  _add_ros2_test(test_onboard_agv_batch_service  test/unit/test_unit_onboard_agv_batch_service.cpp)
  _add_ros2_test(test_offboard_agv_service       test/unit/test_unit_offboard_agv_service.cpp)
  _add_ros2_test(test_offboard_agv_batch_service test/unit/test_unit_offboard_agv_batch_service.cpp)
  _add_ros2_test(test_get_loaded_map_service     test/unit/test_unit_get_loaded_map_service.cpp)
  _add_ros2_test(test_resume_mode_cancelled_queue_service
                                                 test/unit/test_unit_resume_mode_cancelled_queue_service.cpp)
  _add_ros2_test(test_discard_mode_cancelled_queue_service
                                                 test/unit/test_unit_discard_mode_cancelled_queue_service.cpp)
  _add_ros2_test(test_get_master_broker_status_service
                                                 test/unit/test_unit_get_master_broker_status_service.cpp)
  _add_ros2_test(test_assignment_result_publisher
                                                 test/unit/test_unit_assignment_result_publisher.cpp)
  _add_ros2_test(test_assign_order_request_subscriber
                                                 test/unit/test_unit_assign_order_request_subscriber.cpp)
  _add_ros2_test(test_fleet_roster_subscriber
                                                 test/unit/test_unit_fleet_roster_subscriber.cpp)
  _add_ros2_test(test_master_connection_publisher
                                                 test/unit/test_unit_master_connection_publisher.cpp)

  # Pure function — no DDS env needed.
  ament_add_gmock(test_order_status_builder test/unit/test_unit_order_status_builder.cpp)
  target_link_libraries(test_order_status_builder
    ${PROJECT_NAME}_lib
    rclcpp::rclcpp
    ${vda5050_interfaces_TARGETS}
  )
  set_target_properties(test_order_status_builder PROPERTIES
    BUILD_RPATH_USE_ORIGIN TRUE
    INSTALL_RPATH_USE_LINK_PATH TRUE)

  # Pure function — no DDS env needed.
  ament_add_gmock(test_pose_view_builder test/unit/test_unit_pose_view_builder.cpp)
  target_link_libraries(test_pose_view_builder
    ${PROJECT_NAME}_lib
    rclcpp::rclcpp
    ${vda5050_interfaces_TARGETS}
  )
  set_target_properties(test_pose_view_builder PROPERTIES
    BUILD_RPATH_USE_ORIGIN TRUE
    INSTALL_RPATH_USE_LINK_PATH TRUE)

  # Pure header-only helper — no DDS env, no library link needed.
  ament_add_gmock(test_ros2_topic_naming test/unit/test_unit_ros2_topic_naming.cpp)
  target_include_directories(test_ros2_topic_naming PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

ament_package()
