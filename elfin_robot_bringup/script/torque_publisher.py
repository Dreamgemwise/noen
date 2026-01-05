#!/usr/bin/env python
# -*- coding: utf-8 -*-  # 添加编码声明，支持中文等非ASCII字符

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def publish_torque():
    rospy.init_node('torque_publisher', anonymous=True)
    pub = rospy.Publisher('/elfin_arm_controller/command', JointTrajectory, queue_size=10)
    rate = rospy.Rate(100)  # 100Hz发布频率
    
    # 初始化消息
    msg = JointTrajectory()
    msg.header.frame_id = ''
    msg.joint_names = ['elfin_joint1', 'elfin_joint2', 'elfin_joint3', 
                       'elfin_joint4', 'elfin_joint5', 'elfin_joint6']
    
    # 创建轨迹点（仅用力矩控制，其他字段设为0）
    point = JointTrajectoryPoint()
    point.time_from_start = rospy.Duration(0.1)  # 100ms
    point.positions = [0.0] * 6
    point.velocities = [0.0] * 6
    point.accelerations = [0.0] * 6
    point.effort = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 关节1力矩1Nm
    msg.points = [point]
    
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_torque()
    except rospy.ROSInterruptException:
        pass
