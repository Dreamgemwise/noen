
# 先编译
catkin clean -y && catkin_make


# 首先，连接硬件机器人
```
roslaunch elfin_robot_bringup elfin3_bringup.launch

sudo chrt 10 bash
roslaunch elfin_robot_bringup elfin_ros_control.launch

roslaunch elfin3_moveit_config moveit_planning_execution.launch

roslaunch elfin_basic_api elfin_basic_api.launch
```

**上电!!!**

# 接下来，位姿调整



- 运动到初始位姿

```
roslaunch elfin_robot_bringup to_init_pose.launch
```

- 回到原点

```
roslaunch elfin_robot_bringup to_zero_api.launch 
```

# 开启最简单的sint力矩控制 - 单轴


**注意：未锁轴时，操作此步骤，必须回到原点**
```
roslaunch elfin_robot_bringup sintTorqueController.launch
```

```
# 锁轴 - 不行，抱闸必先去使能，使能自动松闸

## 先确保机器人已断使能（或至少速度＝0），再执行 ↓

## 锁 Axis 3、4   —— slave3
rosservice call /elfin_module_close_brake_slave3 "data: true"

## 锁 Axis 5、6   —— slave4
rosservice call /elfin_module_close_brake_slave4 "data: true"



# 仅给轴1，2使能，其他轴保持在抱闸的状态

rosservice call /elfin_module_enable_slave2 "data: true"
```



# 开启FT传感器
```
 sudo setcap cap_net_raw,cap_net_admin+ep /home/triyi/catkin_ws/devel/lib/ftdata/ftdata

 rosrun ftdata ftdata

```

# 开启力矩控制控制器

```
roslaunch elfin_robot_bringup mytorque_controller.launch 
```