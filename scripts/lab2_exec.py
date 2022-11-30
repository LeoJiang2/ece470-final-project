#!/usr/bin/env python

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
from lab2_header import *
from lab4_func import *
from numpy import pi
from blob_search import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])
xw_yw_R = []
xw_yw_G = []

yaw = 0
# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False
image_shape_define = False

p1 = [295, 50, 35, 0]
p2 = [295, 50, 100, 0]
p3 = [295, 200, 200, 0]
############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def input_callback(msg):

    global digital_in_0
    digital_in_0 = msg.DIGIN
    digital_in_0 = digital_in_0 & 1



############### Your Code End Here ###############


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0
    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    # rospy.loginfo(driver_msg)
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):
        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            #rospy.loginfo("Goal is reached!")

        loop_rate.sleep()
        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel):

    """
    start_xw_yw_zw: where to pick up a block in global coordinates
    target_xw_yw_zw: where to place the block in global coordinates

    hint: you will use lab_invk(), gripper(), move_arm() functions to
    pick and place a block

    """
    # ========================= Student's code starts here =========================
    
    above_start_thetas = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], 60, 0)
    start_thetas = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], 35, 0)
    target_thetas = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2], 0)
    above_target_thetas = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2]+20, 0)


    time.sleep(1.0)
    error = 0
    move_arm(pub_cmd, loop_rate, above_start_thetas, vel, accel)
    print(1)
    move_arm(pub_cmd, loop_rate, start_thetas, vel, accel)
    print(2)
    gripper(pub_cmd, loop_rate, suction_on)
    print(3)
    time.sleep(1.0)
    if digital_in_0 == 0:
        gripper(pub_cmd, loop_rate, suction_off)
        print("Block is missing!")
        return error

    move_arm(pub_cmd, loop_rate, above_start_thetas, vel, accel)
    move_arm(pub_cmd, loop_rate, above_target_thetas, vel, accel)
    move_arm(pub_cmd, loop_rate, target_thetas, vel, accel)
    gripper(pub_cmd, loop_rate, suction_off)
    move_arm(pub_cmd, loop_rate, above_target_thetas, vel, accel)
    return error

class ImageConverter:

    def __init__(self, SPIN_RATE):

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/image_converter/output_video", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/cv_camera_node/image_raw", Image, self.image_callback)
        self.loop_rate = rospy.Rate(SPIN_RATE)

        # Check if ROS is ready for operation
        while(rospy.is_shutdown()):
            print("ROS is shutdown!")


    def image_callback(self, data):

        global xw_yw_R # store found green blocks in this list

        try:
          # Convert ROS image to OpenCV image
            raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image = cv2.flip(raw_image, -1)
        cv2.line(cv_image, (0,50), (640,50), (0,0,0), 5)

        # You will need to call blob_search() function to find centers of green blocks
        # and yellow blocks, and store the centers in xw_yw_G & xw_yw_Y respectively.

        # If no blocks are found for a particular color, you can return an empty list,
        # to xw_yw_G or xw_yw_Y.

        # Remember, xw_yw_G & xw_yw_Y are in global coordinates, which means you will
        # do coordinate transformation in the blob_search() function, namely, from
        # the image frame to the global world frame.

        xw_yw_R = blob_search(cv_image, "red")
        # xw_yw_G = blob_search(cv_image, "green")
        # print(xw_yw_G)

############### Your Code End Here ###############


def main():

    global home
    global Q
    global SPIN_RATE
    global xw_yw_R
    global xw_yw_G

    # Parser
    parser = argparse.ArgumentParser(description='Please specify if using simulator or real robot')
    parser.add_argument('--simulator', type=str, default='True')
    args = parser.parse_args()

    # Initialize rospack
    rospack = rospkg.RosPack()
    # Get path to yaml
    lab2_path = rospack.get_path('lab2pkg_py')
    yamlpath = os.path.join(lab2_path, 'scripts', 'lab2_data.yaml')

    with open(yamlpath, 'r') as f:
        try:
            # Load the data as a dict
            data = yaml.load(f)
            if args.simulator.lower() == 'true':
                Q = data['sim_pos']
            elif args.simulator.lower() == 'false':
                Q = data['real_pos']
            else:
                print("Invalid simulator argument, enter True or False")
                sys.exit()
            
        except:
            print("YAML not found")
            sys.exit()

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function

    sub_gripper = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)





    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    vel = 4.0
    accel = 4.0
    ic = ImageConverter(SPIN_RATE)
    time.sleep(5)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input

    Rblock1 = xw_yw_R[0]
    move_arm(pub_command, loop_rate, home, 4.0, 4.0)
    move_block(pub_command, loop_rate,[Rblock1[0]*1000, Rblock1[1]*1000, 35], p2, vel, accel)
    # move_block(pub_command, loop_rate,[Gblock1[0], Gblock1[1], 0], p2, vel, accel)

    # while(loop_count > 0):

    #     # move_arm(pub_command, loop_rate, home, 4.0, 4.0)
    #     # rospy.loginfo("Sending goal 1 ...")
    #     move_arm(pub_command, loop_rate, p1, 4.0, 4.0)
    #     rospy.loginfo(digital_in_0)
    #     gripper(pub_command, loop_rate, suction_on)
    #     # Delay to make sure suction cup has grasped the block
    #     time.sleep(1.0)
        
    #     rospy.loginfo("Sending goal 2 ...")
    #     move_arm(pub_command, loop_rate, p2, 4.0, 4.0)
    #     move_arm(pub_command, loop_rate, p3, 4.0, 4.0)
    #     rospy.loginfo(digital_in_0)
    #     loop_count = loop_count - 1

        
    # gripper(pub_command, loop_rate, suction_off)



    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
