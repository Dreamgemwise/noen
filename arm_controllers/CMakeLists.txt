cmake_minimum_required(VERSION 2.8.3)
project(arm_controllers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 选用系统 g++-9
set(CMAKE_C_COMPILER gcc-9)
set(CMAKE_CXX_COMPILER g++-9)

find_package(catkin REQUIRED COMPONENTS
  angles
  controller_interface
  control_msgs
  urdf
  kdl_parser
  realtime_tools
  dynamic_reconfigure
  elfin_hardware_interface
  # pinocchio  # 注释掉原来的find_package
)

# 使用pkg-config查找Pinocchio
find_package(PkgConfig REQUIRED)
pkg_check_modules(PINOCCHIO REQUIRED pinocchio)

# Dynamics reconfigure
generate_dynamic_reconfigure_options(
  cfg/PassivityControllerParams.cfg
)

# message
add_message_files(FILES ControllerJointState.msg)
generate_messages(DEPENDENCIES std_msgs)

# include
include_directories(
  ${PINOCCHIO_INCLUDE_DIRS}  # 添加Pinocchio头文件目录
  include 
  ${Boost_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/msg/include 
  ${PROJECT_SOURCE_DIR}/include
  
)

catkin_package(
  CATKIN_DEPENDS
    angles
    controller_interface
    control_msgs
    urdf
    kdl_parser
    realtime_tools
    dynamic_reconfigure
    elfin_hardware_interface
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
  src/time_delay_controller.cpp
  src/gravity_comp_controller.cpp
  src/computed_torque_controller.cpp
  src/computed_torque_clik_controller.cpp
  src/adaptive_impedance_controller.cpp
  src/passivity_controller.cpp

  
  src/myprogram/newparam_regulating20250721-v1.cpp
  src/myprogram/to_init_pose.cpp
  src/myprogram/sint_torque_control.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PINOCCHIO_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)




