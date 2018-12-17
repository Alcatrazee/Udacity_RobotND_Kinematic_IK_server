#!/usr/bin/env python2

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from math import *
import numpy as np
from numpy import linalg
from sensor_msgs.msg import JointState
from kuka_arm.srv import Get_pos


def Get_homogenous_Rotation_matrix(axis, theta):

    if axis == 'x':
        rotation_matrix = np.matrix([[1, 0, 0, 0],
                                     [0, np.cos(theta), -np.sin(theta), 0],
                                     [0, np.sin(theta), np.cos(theta), 0]])

    elif axis == 'y':
        rotation_matrix = np.matrix([[np.cos(theta), 0, np.sin(theta), 0],
                                     [0, 1, 0, 0],
                                     [-np.sin(theta), 0, np.cos(theta), 0]])
    elif axis == 'z':
        rotation_matrix = np.matrix([[np.cos(theta), -np.sin(theta), 0, 0],
                                     [np.sin(theta), np.cos(theta), 0, 0],
                                     [0, 0, 1, 0]])

    return np.vstack((rotation_matrix, np.matrix([[0, 0, 0, 1]])))


def Get_homogenous_position_offset(x_offset, z_offset):
    translation_matrix = np.matrix([[1, 0, 0, x_offset],
                                    [0, 1, 0, 0],
                                    [0, 0, 1, z_offset],
                                    [0, 0, 0, 1]])
    return translation_matrix


def Get_Transformation_matrix(Rx, Tx, Rz, Tz):
    return Rx*Tx*Rz*Tz


def Get_DH_Param_Matrix(DH_Param, i):
    Rx = Get_homogenous_Rotation_matrix('x', DH_Param['alpha'+str(i-1)])
    Tx = Get_homogenous_position_offset(DH_Param['a'+str(i-1)], 0)
    Rz = Get_homogenous_Rotation_matrix('z', DH_Param['theta'+str(i)])
    Tz = Get_homogenous_position_offset(0, DH_Param['d'+str(i)])
    return Rx, Tx, Rz, Tz

def d2r(deg):
    return (deg*np.pi)/180


def r2d(rad):
    return rad*180/np.pi

def Get_cost(solution,current_joint_position):
    # cost table w/rad
    Cost_table = {'t1':30,'t2':25,'t3':25,'t4':10,'t5':10,'t6':10}
    cost = np.zeros((32,1))
    each_cost = np.zeros((6,1))
    for i in range(32):
        if(solution[0,i]!=0):
            each_cost[0] = abs(solution[1,i]-current_joint_position[0])*Cost_table['t1']
            each_cost[1] = abs(solution[2,i]-current_joint_position[1])*Cost_table['t2']
            each_cost[2] = abs(solution[3,i]-current_joint_position[2])*Cost_table['t3']
            each_cost[3] = abs(solution[4,i]-current_joint_position[3])*Cost_table['t4']
            each_cost[4] = abs(solution[5,i]-current_joint_position[4])*Cost_table['t5']
            each_cost[5] = abs(solution[6,i]-current_joint_position[5])*Cost_table['t6']
            cost[i] = sum(each_cost)
        else:
            cost[i] = 10000000
    return cost        


    

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        # Your FK code here
        # Create symbols
        #
        cost_list = []
        theta1 = 0
        theta2 = 0
        theta3 = 0
        theta4 = 0
        theta5 = 0
        theta6 = 0
        theta7 = 0
        #
        # Create Modified DH parameters
        #
        DH_Param = {'alpha0': 0, 'a0': 0, 'd1': 0.75, 'theta1': theta1,
                    'alpha1': -np.pi/2, 'a1': 0.35, 'd2': 0, 'theta2': theta2-np.pi/2,
                    'alpha2': 0, 'a2': 1.25, 'd3': 0, 'theta3': theta3,
                    'alpha3': -np.pi/2, 'a3': -0.054, 'd4': 1.5, 'theta4': theta4,
                    'alpha4': np.pi/2, 'a4': 0, 'd5': 0, 'theta5': theta5,
                    'alpha5': -np.pi/2, 'a5': 0, 'd6': 0, 'theta6': theta6,
                    'alpha6': 0, 'a6': 0, 'd7': 0.303, 'theta7': theta7, }

        Limit = {'t1_max': d2r(185), 't1_min': d2r(-185),
                 't2_max': d2r(85), 't2_min': d2r(-45),
                 't3_max': d2r(155-90), 't3_min': d2r(-210),
                 't4_max': d2r(350), 't4_min': d2r(-350),
                 't5_max': d2r(125), 't5_min': d2r(-125),
                 't6_max': d2r(350), 't6_min': d2r(-350)}
        #
        # Define Modified DH Transformation matrix
        #
        [Rx1, Tx1, Rz1, Tz1] = Get_DH_Param_Matrix(DH_Param, 1)
        [Rx2, Tx2, Rz2, Tz2] = Get_DH_Param_Matrix(DH_Param, 2)
        [Rx3, Tx3, Rz3, Tz3] = Get_DH_Param_Matrix(DH_Param, 3)
        [Rx4, Tx4, Rz4, Tz4] = Get_DH_Param_Matrix(DH_Param, 4)
        [Rx5, Tx5, Rz5, Tz5] = Get_DH_Param_Matrix(DH_Param, 5)
        [Rx6, Tx6, Rz6, Tz6] = Get_DH_Param_Matrix(DH_Param, 6)
        #
        # Create individual transformation matrices
        #
        T_01 = Get_Transformation_matrix(Rx1, Tx1, Rz1, Tz1)
        T_12 = Get_Transformation_matrix(Rx2, Tx2, Rz2, Tz2)
        T_23 = Get_Transformation_matrix(Rx3, Tx3, Rz3, Tz3)
        T_34 = Get_Transformation_matrix(Rx4, Tx4, Rz4, Tz4)
        T_45 = Get_Transformation_matrix(Rx5, Tx5, Rz5, Tz5)
        T_56 = Get_Transformation_matrix(Rx6, Tx6, Rz6, Tz6)
        T_67 = Get_homogenous_position_offset(DH_Param['a6'], DH_Param['d7'])
        # Get transformation matrix from base to end-effector
        T_07 = T_01*T_12*T_23*T_34*T_45*T_56*T_67
        # split matrix
        R_07 = T_07[0:3, 0:3]
        # for urdf file
        R_corr = R_07
    #
    # Extract rotation matrices from the transformation matrices
    #

    #
    ###
        
        # Initialize service response
        joint_trajectory_list = []
        # Get current joint state in order to have a better performance
        Joint_state_server = rospy.ServiceProxy('get_current_joint_state',Get_pos)
        resp = Joint_state_server(1)
        current_joint_state = resp.JointState
        current_joint_position = current_joint_state.position
        RPY_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
            RPY_list.append([roll,pitch,yaw])
            # Your IK code here
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            #
            P_EE = np.matrix([[px], [py], [pz]])
            # orientation in base frame
            R_rpy = Get_homogenous_Rotation_matrix(
                'z', yaw)*Get_homogenous_Rotation_matrix('y', pitch)*Get_homogenous_Rotation_matrix('x', roll)
            R_rpy = R_rpy[0:3, 0:3]*R_corr
            
            l3 = sqrt(DH_Param['a3']**2+DH_Param['d4']**2)    
            phi = atan2(-DH_Param['a3'], DH_Param['d4'])
            # wrist center position and orientation
            WC_P = P_EE - (DH_Param['d7'])*R_rpy*np.matrix([[0], [0], [1]])
            solution = np.zeros((7, 32))
            solution[0,:] = np.ones((1,32))
            for solution_num in range(32):
                # calculate theta1
                if(solution_num < 16):
                    solution[1, solution_num] = DH_Param['theta1'] = theta1 = atan2(
                        WC_P[1], WC_P[0])
                else:
                    temp = atan2(WC_P[1], WC_P[0])
                    if(temp > 0):
                        solution[1, solution_num] = DH_Param['theta1'] = theta1 = temp - np.pi
                    elif(temp < 0):
                        solution[1, solution_num] = DH_Param['theta1'] = theta1 = temp + np.pi
                    if(solution[1, solution_num] > Limit['t1_max'] or solution[1, solution_num] < Limit['t1_min']):
                        solution[0, solution_num] = False
                        
                if(solution_num == 0 or solution_num == 16):
                    [Rx1, Tx1, Rz1, Tz1] = Get_DH_Param_Matrix(DH_Param, 1)
                    T_01 = Get_Transformation_matrix(Rx1, Tx1, Rz1, Tz1)
                
                #R_01 = T_01[0:3, 0:3]

                # Get position of joint2 for convenient
                    P_12 = np.matrix([[DH_Param['a1']], [0], [DH_Param['d2']], [1]])
                    J2_pos = T_01*P_12
                    P_02 = J2_pos[:3]

                #
                # Calculate joint angles using Geometric IK method
                
                # distance between WC and Joint2
                    WC2J2 = linalg.norm(WC_P-P_02)
                # angle between J2 and WC
                    if(solution_num==0):
                        angle_J2_WC = atan2(WC_P[2]-DH_Param['d1'],sqrt(WC_P[0]**2+WC_P[1]**2)-DH_Param['a1'])
                    elif(solution_num==16):
                        angle_J2_WC = atan2(WC_P[2]-DH_Param['d1'],sqrt(WC_P[0]**2+WC_P[1]**2)+DH_Param['a1'])
                    try:
                        angle_a = acos((DH_Param['a2']**2 + WC2J2**2 - l3**2)/(2*DH_Param['a2']*WC2J2))
                        angle_b = acos((l3**2+DH_Param['a2']**2-WC2J2**2)/(2*DH_Param['a2']*l3))
                    except ValueError:
                        if(solution_num==0):
                            for z in range(16):
                                solution[0, z] = False
                            solution_num+=16
                            continue
                        elif(solution_num==16):
                            for z in range(16):
                                solution[0, z+16] = False
                            solution_num+=16
                            break
                            
                if(0<=solution_num<8):
                    temp2 = np.round(np.pi/2 - angle_a - angle_J2_WC, 6)
                    temp3 = np.round(np.pi/2 - phi - angle_b, 6)
                elif(8<=solution_num<16):
                    temp2 = np.round(np.pi/2 + angle_a - angle_J2_WC, 6)
                    temp3 = np.round(angle_b - 3*np.pi/2 - phi,6)
                elif(16<=solution_num<24):
                    temp2 = np.round(angle_J2_WC + angle_a - np.pi/2, 6)
                    temp3 = np.round(angle_b - phi - 3*np.pi/2,6)
                elif(24<=solution_num<32):
                    temp2 = np.round(angle_J2_WC - np.pi/2 - angle_a, 6)
                    temp3 = np.round(angle_b - phi - np.pi, 6)
                    
                if(temp2>Limit['t2_max'] or temp2<Limit['t2_min'] or temp3>Limit['t3_max'] or temp3<Limit['t3_min'] ):
                    solution[0, solution_num] = False

                solution[2, solution_num] = theta2 = temp2
                solution[3, solution_num] = theta3 = temp3

                DH_Param['theta2'] = theta2 - np.pi/2
                DH_Param['theta3'] = theta3

                [Rx2, Tx2, Rz2, Tz2] = Get_DH_Param_Matrix(DH_Param, 2)
                [Rx3, Tx3, Rz3, Tz3] = Get_DH_Param_Matrix(DH_Param, 3)

                T_12 = Get_Transformation_matrix(Rx2, Tx2, Rz2, Tz2)
                T_23 = Get_Transformation_matrix(Rx3, Tx3, Rz3, Tz3)

                T_03 = T_01*T_12*T_23
                # middle variables
                
                R_03 = T_03[0:3, 0:3]

                R_36 = R_03.T*R_rpy

                for i in range(3):
                    for j in range(3):
                        R_36[i, j] = np.round(R_36[i, j], 6)   

                # nonsingular case:
                if(R_36[0,2]!=0 and R_36[2,2]!=0):
                    sin_5 = np.sqrt(1-R_36[1,2]**2)
                    temp5 = abs(atan2(sin_5, R_36[1,2]))
                    temp4 = atan2(R_36[2, 2], -R_36[0, 2])
                    temp6 = atan2(-R_36[1, 1], R_36[1, 0])
                    if(solution_num%8==0):
                        solution[5, solution_num] = DH_Param['theta5'] = theta5 = temp5
                        solution[4, solution_num] = DH_Param['theta4'] = theta4 = temp4
                        solution[6, solution_num] = DH_Param['theta6'] = theta6 = temp6
                        
                    elif(solution_num%8==1):
                        solution[5, solution_num] = DH_Param['theta5'] = theta5 = temp5
                        solution[4, solution_num] = DH_Param['theta4'] = theta4 = temp4
                        if(temp6>0):
                            solution[6, solution_num] = DH_Param['theta6'] = theta6 = temp6-2*np.pi
                        else:
                            solution[6, solution_num] = DH_Param['theta6'] = theta6 = temp6+2*np.pi

                    elif(solution_num%8==2):
                        solution[5, solution_num] = DH_Param['theta5'] = theta5 = temp5
                        if(temp4>0):
                            solution[4, solution_num] = DH_Param['theta4'] = theta4 = temp4-np.pi*2
                        else:
                            solution[4, solution_num] = DH_Param['theta4'] = theta4 = temp4+np.pi*2
                        solution[6, solution_num] = DH_Param['theta6'] = theta6 = temp6

                    elif(solution_num%8==3):
                        solution[5, solution_num] = DH_Param['theta5'] = theta5 = temp5
                        if(temp4>0):
                            solution[4, solution_num] = DH_Param['theta4'] = theta4 = temp4-np.pi*2
                        else:
                            solution[4, solution_num] = DH_Param['theta4'] = theta4 = temp4+np.pi*2
                        if(temp6>0):
                            solution[6, solution_num] = DH_Param['theta6'] = theta6 = temp6-np.pi*2
                        else:
                            solution[6, solution_num] = DH_Param['theta6'] = theta6 = temp6+np.pi*2
                    #
                    #         
                    elif(solution_num%8==4):
                        solution[5, solution_num] = DH_Param['theta5'] = theta5 = -temp5
                        if(temp4-np.pi>0):
                            solution[4, solution_num] = DH_Param['theta4'] = theta4 = temp4-np.pi
                        else:
                            solution[4, solution_num] = DH_Param['theta4'] = theta4 = temp4+np.pi
                        if(temp6-np.pi>0):
                            solution[6, solution_num] = DH_Param['theta6'] = theta6 = temp6-np.pi
                        else:
                            solution[6, solution_num] = DH_Param['theta6'] = theta6 = temp6+np.pi
                    elif(solution_num%8==5):
                        solution[5, solution_num] = DH_Param['theta5'] = theta5 = -temp5
                        if(temp4-np.pi>0):
                            solution[4, solution_num] = DH_Param['theta4'] = theta4 = temp4-np.pi
                        else:
                            solution[4, solution_num] = DH_Param['theta4'] = theta4 = temp4+np.pi
                        if(temp6-np.pi>0):
                            solution[6, solution_num] = DH_Param['theta6'] = theta6 = temp6-np.pi+pi*2
                        else:
                            solution[6, solution_num] = DH_Param['theta6'] = theta6 = temp6+np.pi-pi*2
                    elif(solution_num%8==6):
                        solution[5, solution_num] = DH_Param['theta5'] = theta5 = -temp5
                        if(temp4-np.pi>0):
                            solution[4, solution_num] = DH_Param['theta4'] = theta4 = temp4-np.pi+pi*2
                        else:
                            solution[4, solution_num] = DH_Param['theta4'] = theta4 = temp4+np.pi-pi*2
                        if(temp6-np.pi>0):
                            solution[6, solution_num] = DH_Param['theta6'] = theta6 = temp6-np.pi
                        else:
                            solution[6, solution_num] = DH_Param['theta6'] = theta6 = temp6+np.pi
                    elif(solution_num%8==7):
                        solution[5, solution_num] = DH_Param['theta5'] = theta5 = -temp5
                        if(temp4-np.pi>0):
                            solution[4, solution_num] = DH_Param['theta4'] = theta4 = temp4-np.pi+pi*2
                        else:
                            solution[4, solution_num] = DH_Param['theta4'] = theta4 = temp4+np.pi-pi*2
                        if(temp6-np.pi>0):
                            solution[6, solution_num] = DH_Param['theta6'] = theta6 = temp6-np.pi+pi*2
                        else:
                            solution[6, solution_num] = DH_Param['theta6'] = theta6 = temp6+np.pi-pi*2
                    


                        
                        
                # singular case:
                elif(R_36[0,2]==0 and R_36[2,2]==0):
                    solution[5, solution_num] = DH_Param['theta5'] = theta5 = 0
                    sum_t4_t6 = atan2(-R_36[2,0],R_36[0,0])
                    if(solution_num==0 or solution_num==2 or solution_num==4 or solution_num==6):
                        solution[4, solution_num] = DH_Param['theta4'] = theta4 = 0
                        solution[6, solution_num] = DH_Param['theta6'] = theta6 = sum_t4_t6-theta4
                    elif(solution_num==1 or solution_num==3 or solution_num==5 or solution_num==7):
                        solution[4, solution_num] = DH_Param['theta4'] = theta4 = sum_t4_t6-theta6
                        solution[6, solution_num] = DH_Param['theta6'] = theta6 = 0

                if(theta4>Limit['t4_max'] or theta4<Limit['t4_min'] or theta5>Limit['t5_max'] or theta5<Limit['t5_min'] or theta6>Limit['t6_max'] or theta6<Limit['t6_min'] ):
                    solution[0, solution_num] = False                                                            
            #
            ###
            # Selet the best solution 
            cost = Get_cost(solution,current_joint_position)
            cost = cost.tolist()
            solution_number = cost.index(min(cost))
            cost_list.append(cost) 
            theta1 = DH_Param['theta1'] = solution[1,solution_number]
            theta2 = DH_Param['theta2'] = solution[2,solution_number]
            theta3 = DH_Param['theta3'] = solution[3,solution_number]
            theta4 = DH_Param['theta4'] = solution[4,solution_number]
            theta5 = DH_Param['theta5'] = solution[5,solution_number]
            theta6 = DH_Param['theta6'] = solution[6,solution_number]

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [
                theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)
            # set current position for next cycle
            current_joint_position = [theta1, theta2, theta3, theta4, theta5, theta6]
            print(solution_number)

        rospy.loginfo("length of Joint Trajectory List: %s" %
                      len(joint_trajectory_list))
        
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print('start the transmitter node first.')
    rospy.wait_for_service('get_current_joint_state')
    print "Ready to receive an IK request"
    rospy.spin()


if __name__ == "__main__":
    IK_server()
