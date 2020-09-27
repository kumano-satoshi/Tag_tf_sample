#!/usr/bin/env python3

import rospy
import numpy as np
import math

from apriltag_ros.msg import AprilTagDetectionArray


class Tag_Sub(object):
    def __init__(self):
        self.tag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.callback)
        self.tag_xyz = np.zeros([2, 3], dtype = "float64")
        self.tag_quaternion = np.zeros([2, 4], dtype = "float64")


    def callback(self, message):
        if len(message.detections) > 0:
            for i in range(len(message.detections)):
                tag_id = message.detections[i].id
                #AprilTagのxyz座標値と姿勢(クオータニオン)を取得
                self.tag_xyz[i][0] = message.detections[i].pose.pose.pose.position.x
                self.tag_xyz[i][1] = message.detections[i].pose.pose.pose.position.y
                self.tag_xyz[i][2] = message.detections[i].pose.pose.pose.position.z
                self.tag_quaternion[i][0] = message.detections[i].pose.pose.pose.orientation.x
                self.tag_quaternion[i][1] = message.detections[i].pose.pose.pose.orientation.y
                self.tag_quaternion[i][2] = message.detections[i].pose.pose.pose.orientation.z
                self.tag_quaternion[i][3] = message.detections[i].pose.pose.pose.orientation.w
             
                print('\n')
                print("tag_id :", tag_id)
                print("tag_xyz :", self.tag_xyz[i])
                print("tag_Quaternion :", self.tag_quaternion[i])
                
        else:
            print("There are no Tags.") 

if __name__ == "__main__":
    rospy.init_node("tag_sample")
    tag_subscriber = Tag_Sub()
    rospy.spin()
