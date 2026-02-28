cmake_minimum_required(VERSION 3.8)
project(ros2_jodell_epg40_050_gripper_rs485_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GripperCommand.msg"
  "srv/Trigger.srv"
  "srv/SetInt32.srv"
  "srv/ChangeBaudRate.srv"
  "srv/SetSlaveID.srv"
  "srv/SetPreset.srv"
  "srv/GetAllPresets.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

# Optional: install msg directory (useful for introspection tools)
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)


ament_package()





