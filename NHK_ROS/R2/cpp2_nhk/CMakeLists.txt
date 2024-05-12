cmake_minimum_required(VERSION 3.8)
project(cpp2_nhk)

find_package(ament_cmake_auto REQUIRED)

#xmlから依存を見つける
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
src/dynamixel_workbench_toolbox/dynamixel_driver.cpp include/dynamixel_workbench_toolbox/dynamixel_driver.h
src/dynamixel_workbench_toolbox/dynamixel_item.cpp include/dynamixel_workbench_toolbox/dynamixel_item.h
src/dynamixel_workbench_toolbox/dynamixel_tool.cpp include/dynamixel_workbench_toolbox/dynamixel_tool.h
src/dynamixel_workbench_toolbox/dynamixel_workbench.cpp include/dynamixel_workbench_toolbox/dynamixel_workbench.h
)

#実行ファイルの生成
ament_auto_add_executable(
  bt_node src/bt_node.cpp include/action_node.hpp include/ros_node.hpp
)
ament_auto_add_executable(
  arm_control_node src/arm_control_node.cpp
)
ament_auto_add_executable(
  arm_dxl_node src/arm_dxl_node.cpp
)
ament_auto_add_executable(
  get_ball_node src/get_ball_node.cpp
)
# ament_auto_add_executable(
#   line_trace_node src/line_trace_node.cpp
# )
ament_auto_add_executable(
  ascend_slope_node src/ascend_slope_node.cpp
)
ament_auto_add_executable(
  move_target_node src/move_target_node.cpp
)
ament_auto_add_executable(
  pan_tilt_node src/pan_tilt_node.cpp
)
ament_auto_add_executable(
  put_ball_node src/put_ball_node.cpp
)
ament_auto_add_executable(
  r2_base_node src/r2_base_node.cpp
)
ament_auto_add_executable(
  select_ball_node src/select_ball_node.cpp
)
# ament_auto_add_executable(
#   joy_control_test src/test/joy_control_test.cpp
# )
# ament_auto_add_executable(
#   pantilt_control_joy src/test/pantilt_control_joy.cpp
# )
# ament_auto_add_executable(
#   test_node1 src/test/test_node1.cpp
# )
# ament_auto_add_executable(
#   test_node2 src/test/test_node2.cpp
# )

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

#構文チェック
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#ライブラリのビルド
ament_auto_package()