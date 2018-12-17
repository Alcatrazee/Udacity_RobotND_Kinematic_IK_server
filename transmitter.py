#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from service_py import *
from kuka_arm.srv import Get_pos
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

position = []


def message_callback(JointState):
    global position
    position = JointState.position
    # print(JointState.position)


def service_callback(req):
    global position
    respon = JointState()
    respon.position = position
    print('answered')
    return respon


if __name__ == "__main__":
    rospy.init_node('transmitter')
    sub = rospy.Subscriber('/joint_states', JointState, message_callback)
    srv = rospy.Service('get_current_joint_state',
                        Get_pos, service_callback)
    print('service started')
    rospy.spin()
