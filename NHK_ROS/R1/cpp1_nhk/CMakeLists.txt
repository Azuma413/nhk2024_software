cmake_minimum_required(VERSION 3.8)
project(cpp1_nhk)

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
  auto_launch_node src/auto_launch_node.cpp
)
ament_auto_add_executable(
  calc_pos_node src/calc_pos_node.cpp
)
ament_auto_add_executable(
  calc_distance_node src/calc_distance_node.cpp
)
ament_auto_add_executable(
  control_node src/control_node.cpp
)
ament_auto_add_executable(
  dynamixel_node src/dynamixel_node.cpp
)
ament_auto_add_executable(
  imu_pub_node src/imu_pub_node.cpp
)
ament_auto_add_executable(
  move_certain_node src/move_certain_node.cpp
)
ament_auto_add_executable(
  r1_base_node src/r1_base_node.cpp
)
ament_auto_add_executable(
  take_rice_node src/take_rice_node.cpp
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

#構文チェック
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#ライブラリのビルド
ament_auto_package()