#!/usr/bin/env python3
import numpy as np
from scipy.linalg import expm
# from lab4_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix

	w1 = np.array([0,0,1])
	w2 = np.array([0,1,0])
	w3 = np.array([0,1,0])
	w4 = np.array([0,1,0])
	w5 = np.array([1,0,0])
	w6 = np.array([0,1,0])

	#skew matrix
	wx = np.array([[0,0,0],[0,0,-1],[0,1,0]])
	wy = np.array([[0,0,1],[0,0,0],[-1,0,0]])
	wz = np.array([[0,-1,0],[1,0,0],[0,0,0]])

	q1 = np.array([-150,150,10])
	q2 = np.array([-150,270,162])
	q3 = np.array([94,270,162])
	q4 = np.array([307,177,162])
	q5 = np.array([307,260,162])
	q6 = np.array([390,260,162])


	v1 = np.array([np.cross(-w1,q1)])
	v2 = np.array([np.cross(-w2,q2)])
	v3 = np.array([np.cross(-w3,q3)])
	v4 = np.array([np.cross(-w4,q4)])
	v5 = np.array([np.cross(-w5,q5)])
	v6 = np.array([np.cross(-w6,q6)])

	# print(v1)
	# print(v2)
	# print(v3)
	# print(v4)
	# print(v5)
	# print(v6)

	s1 = np.concatenate((np.concatenate((wz,v1.T), axis=1), np.array([np.zeros((4),dtype=int)])), axis=0)
	s2 = np.concatenate((np.concatenate((wy,v2.T), axis=1), np.array([np.zeros((4),dtype=int)])), axis=0)
	s3 = np.concatenate((np.concatenate((wy,v3.T), axis=1), np.array([np.zeros((4),dtype=int)])), axis=0)
	s4 = np.concatenate((np.concatenate((wy,v4.T), axis=1), np.array([np.zeros((4),dtype=int)])), axis=0)
	s5 = np.concatenate((np.concatenate((wx,v5.T), axis=1), np.array([np.zeros((4),dtype=int)])), axis=0)
	s6 = np.concatenate((np.concatenate((wy,v6.T), axis=1), np.array([np.zeros((4),dtype=int)])), axis=0)

	S = np.array([s1,s2,s3,s4,s5,s6])
	M = np.array([[0,-1,0,390],[0,0,-1,401],[1,0,0,215.5],[0,0,0,1]])


	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
# def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

# 	# Initialize the return_value
# 	return_value = [None, None, None, None, None, None]

# 	print("Foward kinematics calculated:\n")

# 	# =================== Your code starts here ====================#
# 	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])

# 	M, S = Get_MS()

# 	T = expm(S[0]*theta1)@expm(S[1]*theta2)@expm(S[2]*theta3)@expm(S[3]*theta4)@expm(S[4]*theta5)@expm(S[5]*theta6)@M

# 	print(str(T) + "\n")

# 	# ==============================================================#

# 	return_value[0] = theta1 + PI
# 	return_value[1] = theta2
# 	return_value[2] = theta3
# 	return_value[3] = theta4 - (0.5*PI)
# 	return_value[4] = theta5
# 	return_value[5] = theta6

# 	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#

	theta1 = 0.0
	theta2 = 0.0
	theta3 = 0.0
	theta4 = 0.0
	theta5 = 0.0
	theta6 = 0.0

	yaw = np.radians(yaw_WgripDegree)

# Step 1 
	xgrip = xWgrip + 150
	ygrip = yWgrip - 150
	zgrip = zWgrip - 10

# Step 2
	xcen = xgrip - 53.5 * np.cos(yaw)
	ycen = ygrip - 53.5 * np.sin(yaw)
	zcen = zgrip

# Step 3
	theta1 = np.arctan2(ycen, xcen) - np.arcsin(110/np.sqrt(xcen**2 + ycen**2))

# Step 4
	theta6 = theta1 + np.radians(90) - yaw

# Step 5
	x3end = xcen + 110 * np.sin(theta1) - 83 * np.cos(theta1)
	y3end = ycen - 110 * np.cos(theta1) - 83 * np.sin(theta1)
	z3end = zcen + 141



# Step 6
	d_y = z3end - 152
	d_x = np.sqrt(x3end**2 + y3end**2)
	l3 = 244
	l5 = 213

	ref_hypot = np.sqrt(d_x**2 + d_y**2)
	ref_theta2 = np.arctan2(d_y, d_x)
	ref_theta4 = np.arctan2(d_x, d_y)
	ref_theta2_2 = np.arccos((l5**2 - l3**2 - ref_hypot**2) / (-2 * l3 * ref_hypot))
	ref_theta4_2 = np.arccos((l3**2 - l5**2 - ref_hypot**2) / (-2 * l5 * ref_hypot))
	ref_theta3_2 = np.arccos((ref_hypot**2 - l5**2 - l3**2) / (-2 * l5 * l3))
	theta2 = -(ref_theta2 + ref_theta2_2)
	theta3 = np.pi - ref_theta3_2
	theta4 = -(ref_theta4 + ref_theta4_2 - (np.pi/2))

	theta5 = -np.pi/2

	print(theta1, theta2, theta3, theta4, theta5, theta6)

	# ==============================================================#
	return [theta1+np.pi, theta2, theta3, theta4 - (0.5*np.pi), theta5, theta6]