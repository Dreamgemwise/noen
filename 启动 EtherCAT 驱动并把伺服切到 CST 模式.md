------

## 1. 启动 EtherCAT 驱动并把伺服切到 **CST** 模式

### 1-1 先验证当前模式

```bash
# 终端① – 启动硬件
sudo chrt 10 bash
roslaunch elfin_robot_bringup elfin_ros_control.launch      # EtherCAT V1
# 真机如果是 V2 / V3，从 `_v2.launch` 或 `_v3.launch` 里选
```

- 在第二个终端调用服务读取 **RxPDO**：

```bash
rosservice call /elfin_ros_control/elfin/get_rxpdo "{}"
```

服务会把所有从站 RxPDO 打印出来，其中应包含对象 `0x6060`（Modes of operation）。若值 ≠ `0x0F`，说明驱动没有处在 **Cyclic Synchronous Torque (CST)**【 turn6file11 L1-L5 】。

### 1-2 一次性把驱动切为 CST1

仓库的 EtherCAT 客户端源文件

```
elfin_ethercat_driver_v2/src/elfin_ethercat_client_v2.cpp
```

在 `ElfinEtherCATClient::configSlave()`（约 480 行附近）里已经对 **0x6060** 写入模式，默认跟随控制器类型。如果你想**强制** CST，可在该函数顶部加入：

```cpp
writeSDO(slave_no_, 0x6060, 0x00, (uint8_t)0x0F);   // 0x0F = CST
```

> ✅ **路径不变**，改完重新编译：
>
> ```bash
> catkin_make -DCMAKE_BUILD_TYPE=Release
> ```
>
> 然后重启 `elfin_ros_control.launch`。

------

## 2. 给 ROS 控制器换成 **

在 `elfin_robot_bringup/launch/elfin_ros_control.launch` 里，控制器列表默认加载 `position_controllers/JointTrajectoryController`。

1. 复制一份模板成 `elfin_torque_arm_controller.yaml`，改成

   ```yaml
   type: effort_controllers/JointTrajectoryController
   joints: [elfin_joint1, elfin_joint2, elfin_joint3, elfin_joint4, elfin_joint5, elfin_joint6]
   gains: {elfin_joint1: {p: 0.0, d: 0.0, i: 0.0}, ...}   # 先全部零，纯力矩直通
   ```

2. 在同一 launch 文件 `<rosparam>` 段里加载上面的 YAML，并把 `<controller_list>` 的 `hardware_interface` 字段改成 `EffortJointInterface`。

重新 `roslaunch` 后，**驱动将在启动序列里自动检测到 Effort 接口并保持 0x6060=0x0F**。

------

## 3. 发送目标力矩 & 监测

### 3-1 读取实际力矩

调用

```bash
rosservice call /elfin_ros_control/elfin/get_txpdo "{}"
```

返回列表里应看到 `0x6077 Actual Torque`，变化与目标一致【 turn5file0 L2-6 】。若值随指令同步变化，即证明 CST 生效。

------

