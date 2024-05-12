cmake_minimum_required(VERSION 3.8)
project(actuator_custom_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HandAirStatus.msg"
  "msg/ShootAirStatus.msg"
  "msg/ShootRoller.msg"
  "msg/ArmControl.msg"
  "msg/SetPosition.msg"
  "srv/GetPosition.srv"
  "srv/GetSiloState.srv"
  "srv/SetTableRad.srv"
  "srv/StartCalcPos.srv"
  "action/CallAscendSlope.action"
  "action/CallLineTrace.action"
  "action/CallReturnLine.action"
  "action/CallGetBall.action"
  "action/CallPutBall.action"
  "action/SetMoveTarget.action"
  "action/CallTakeRice.action"
  "action/CallAutoLaunch.action"
  "action/CallMoveCertain.action"
  "action/GetBallCoord.action"
)
ament_auto_package()
