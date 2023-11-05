#!/usr/bin/env python3
# coding=utf8
from __future__ import print_function, division, absolute_import
from __future__ import print_function, division, absolute_import

import _thread
import copy
import time

import numpy as np
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry

# from nav_msgs.msg import Path

cur_odom_to_baselink = None
cur_map_to_odom = None


def pose_to_mat(pose_msg):
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
    )


def transform_fusion():
    global cur_odom_to_baselink, cur_map_to_odom

    br = tf.TransformBroadcaster()
    while True:
        time.sleep(1 / FREQ_PUB_LOCALIZATION)

        # TODO 这里注意线程安全
        cur_odom = copy.copy(cur_odom_to_baselink)
        if cur_map_to_odom is not None:
            T_map_to_odom = pose_to_mat(cur_map_to_odom)
        else:
            T_map_to_odom = np.eye(4)

        br.sendTransform(tf.transformations.translation_from_matrix(T_map_to_odom),
                         tf.transformations.quaternion_from_matrix(T_map_to_odom),
                         rospy.Time.now(),
                         'camera_init', 'map')
        if cur_odom is not None:
            # 发布全局定位的odometry
            localization = Odometry()
            T_odom_to_base_link = pose_to_mat(cur_odom)
            # 这里T_map_to_odom短时间内变化缓慢 暂时不考虑与T_odom_to_base_link时间同步
            T_map_to_base_link = np.matmul(T_map_to_odom, T_odom_to_base_link)
            xyz = tf.transformations.translation_from_matrix(T_map_to_base_link)
            quat = tf.transformations.quaternion_from_matrix(T_map_to_base_link)
            localization.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
            localization.twist = cur_odom.twist

            localization.header.stamp = cur_odom.header.stamp
            localization.header.frame_id = 'map'
            localization.child_frame_id = 'body'
            # rospy.loginfo_throttle(1, '{}'.format(np.matmul(T_map_to_odom, T_odom_to_base_link)))
            pub_localization.publish(localization)

def cb_save_cur_odom(odom_msg):
    global cur_odom_to_baselink
    cur_odom_to_baselink = odom_msg


def cb_save_map_to_odom(odom_msg):
    global cur_map_to_odom
    cur_map_to_odom = odom_msg


if __name__ == '__main__':
    # tf and localization publishing frequency (HZ)
    FREQ_PUB_LOCALIZATION = 50

    rospy.init_node('transform_fusion')
    rospy.loginfo('Transform Fusion Node Inited...')

    rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1)
    rospy.Subscriber('/map_to_odom', Odometry, cb_save_map_to_odom, queue_size=1)
    # pub_path = rospy.Publisher('/base_path', Path, queue_size=10)
    pub_localization = rospy.Publisher('/localization', Odometry, queue_size=1)

    # 发布定位消息
    _thread.start_new_thread(transform_fusion, ())

    rospy.spin()

# 这段代码是另一个ROS节点，名为 `transform_fusion`，它执行的任务是融合不同坐标系下的位姿信息，然后发布转换后的位姿信息。
#
# 以下是代码的主要部分和功能：
#
# 1. 导入所需的库和模块，包括ROS消息、NumPy、tf（Transform Library）等。
#
# 2. 定义全局变量 `cur_odom_to_baselink`，用于存储当前里程计到基座标系的变换，以及 `cur_map_to_odom` 用于存储地图到里程计的变换。
#
# 3. `pose_to_mat` 函数用于将位姿消息转换为变换矩阵。
#
# 4. `transform_fusion` 函数是主要的逻辑，它在一个循环中不断融合不同坐标系下的位姿信息，然后发布转换后的位姿信息。它首先等待一定时间（由`FREQ_PUB_LOCALIZATION` 定义的频率），然后融合当前 `cur_odom_to_baselink` 与 `cur_map_to_odom` 来计算全局定位的位姿信息，并使用 `tf.TransformBroadcaster()` 来发布这个变换。
#
# 5. `cb_save_cur_odom` 和 `cb_save_map_to_odom` 是回调函数，用于保存当前里程计和地图到里程计的位姿信息。
#
# 6. 在主函数中，定义了一个名为 `FREQ_PUB_LOCALIZATION` 的常量，初始化ROS节点，设置消息发布者和订阅者，然后启动 `transform_fusion` 函数作为一个独立的线程。
#
# 此代码的主要目标是融合不同坐标系下的位姿信息，将地图到里程计的位姿信息发布到 `/localization` 主题。这对于在机器人上实现全局定位非常重要，因为它将不同坐标系中的位姿信息转换为地图坐标系下的全局位姿信息，从而为机器人提供全局定位支持。
