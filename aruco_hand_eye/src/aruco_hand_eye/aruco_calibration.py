#!/usr/bin/env python

import rospy
import sys
import numpy as np
from std_msgs.msg import Bool, Int32
from ROS_Socket.srv import get_curr_pos, get_curr_posRequest
from aruco_hand_eye.srv import hand_eye_calibration, hand_eye_calibrationRequest
from geometry_msgs.msg import Transform

def get_fb():
    rospy.wait_for_service('get_curr_pos_server')
    try:
        get_endpos = rospy.ServiceProxy(
            'get_curr_pos_server',
            get_curr_pos
        )
        req = get_curr_posRequest()
        req.cmd = 'hi'
        res = get_endpos(req)
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def hand_eye_client(req):
    rospy.wait_for_service('/camera/hand_eye_calibration')
    try:
        hand_eye = rospy.ServiceProxy('/camera/hand_eye_calibration', hand_eye_calibration)
        res = hand_eye(req)
        return res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == '__main__':
    name = 'aa'
    rospy.init_node('aruco_calibration')
    pose = get_fb()
    req = hand_eye_calibrationRequest()
    req.cmd = name
    req.end_trans.translation.x = pose.arm_pose.translation.x
    req.end_trans.translation.y = pose.arm_pose.translation.y
    req.end_trans.translation.z = pose.arm_pose.translation.z
    req.end_trans.rotation.w    = pose.arm_pose.rotation.w   
    req.end_trans.rotation.x    = pose.arm_pose.rotation.x   
    req.end_trans.rotation.y    = pose.arm_pose.rotation.y   
    req.end_trans.rotation.z    = pose.arm_pose.rotation.z   
    hand_eye_client(req)
