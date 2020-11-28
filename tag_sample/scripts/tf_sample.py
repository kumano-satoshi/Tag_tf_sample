#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
import tf

from apriltag_ros.msg import AprilTagDetectionArray


class Tag_Sub(object):
    def __init__(self):
        self.tag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.callback)
        self.TB = tf.TransformBroadcaster()
        self.TL = tf.TransformListener()
        
        self.tag0 = [[] for _ in range(2)]
        self.tag1 = [[] for _ in range(2)]


    def quaternion_to_euler(self, quaternion):
        #クオータニオンからオイラー角へ変換する関数
        euler_radian = tf.transformations.euler_from_quaternion(quaternion)
        return euler_radian


    def euler_to_quaternion(self, euler):
        #オイラー角からクオータニオンへ変換する関数
        quaternion = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
        return quaternion


    def broadcaster(self, xyz_position, quaternion, target_frame, base_frame):
        #座標系の登録関数
        self.TB.sendTransform(xyz_position, quaternion, rospy.Time.now(), target_frame, base_frame)


    def listener(self, base_frame, target_frame):
        #座標系の相対関係取得関数
        (trans, rot) = self.TL.lookupTransform(base_frame, target_frame, rospy.Time(0))
        return trans, rot


    def callback(self, data):
        try:
            for i in range(len(data.detections)):
                tag_p = data.detections[i].pose.pose.pose.position
                tag_q = data.detections[i].pose.pose.pose.orientation
                if data.detections[i].id[0] == 0:
                    self.tag0[0] = [tag_p.x, tag_p.y, tag_p.z]
                    self.tag0[1] = [tag_q.x, tag_q.y, tag_q.z, tag_q.w]
                    #print(self.tag0)
                elif data.detections[i].id[0] == 1:
                    self.tag1[0] = [tag_p.x, tag_p.y, tag_p.z]
                    self.tag1[1] = [tag_q.x, tag_q.y, tag_q.z, tag_q.w]
                    #print(self.tag1)
                #print("\n")
        except:
            print("There are no Tags.")
            
        try:
            tag0_cam_t, tag0_cam_r = self.listener("tag0", "usb_cam")
            print(tag0_cam_t)
            print(tag0_cam_r)
            print("\n")
            
            self.broadcaster([1, 0, 0], [0, 0, 0, 1], "cam_no_tonari", "tag0")
            tag0_cnt_t, tag0_cnt_r = self.listener("usb_cam", "cam_no_tonari")
            print(tag0_cnt_t)
            print(tag0_cnt_r)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(e)




if __name__ == "__main__":
    rospy.init_node("tf_sample")
    tag_sub = Tag_Sub()
    rospy.spin()
