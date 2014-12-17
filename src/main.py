#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory

import cv2
import urllib
import numpy as np
import math

def invkin(xyz):
	"""
	Python implementation of the the inverse kinematics for the crustcrawler
	robot created by Peter. Adjust parameters (d1,a1,a2,d4) accordingly.
	The robot model shown in rviz can be adjusted by editing au_crustcrawler_ax12.urdf
	"""

	d1 = 12.0; # cm (height of 2nd joint)
	a1 = 0.0; # (distance along "y-axis" to 2nd joint)
	a2 = 17.0; # (distance between 2nd and 3rd joints)
	d4 = 28.0; # (distance from 3rd joint to gripper center - all inclusive, ie. also 4th joint)

	oc = xyz; # - d4*R*[0;0;1] ;
	xc = xyz[0]*100.0; yc = xyz[1]*100.0; zc = xyz[2]*100.0;

	q1 = math.atan2(xyz[1],xyz[0])

	r2 = math.pow((xc - a1 * math.cos(q1) ),2) + math.pow(( yc - a1 * math.sin(q1) ) , 2); # radius squared - radius can never be negative, q1 accounts for this..
	s = (zc - d1); # can be negative ! (below first joint height..)
	D = ( r2 + math.pow(s,2) - math.pow(a2 , 2) - math.pow(d4,2))/(2.*a2*d4);   # Eqn. (3.44)

	q3 = math.atan2(-math.sqrt(1-math.pow(D,2)), D); #  Eqn. (3.46)
	q2 = math.atan2(s, math.sqrt(r2)) - math.atan2(d4*math.sin(q3), a2 + d4*math.cos(q3)); # Eqn. (3.47)
	q4 = 0.0

	return q1,q2-math.pi/2,q3,q4

class ActionExampleNode:

	N_JOINTS = 4
	def __init__(self,server_name):
		self.client = actionlib.SimpleActionClient(server_name, FollowJointTrajectoryAction)

		self.joint_positions = []
		self.names =["joint1",
				"joint2",
				"joint3",
				"joint4"
				]
		# the list of xyz points we want to plan
		xyz_positions = [
		[0.2, 0.2, 0.0],
		[0.2, -0.2, 0.0]
		#[0.2, 0.2, 0.2]
		]

		# initial duration
		dur = rospy.Duration(1)

		# construct a list of joint positions by calling invkin for each xyz point
		for p in xyz_positions:
			jtp = JointTrajectoryPoint(positions=invkin(p),velocities=[0.5]*self.N_JOINTS ,time_from_start=dur)
			dur += rospy.Duration(2)
			self.joint_positions.append(jtp)

		self.jt = JointTrajectory(joint_names=self.names, points=self.joint_positions)
		self.goal = FollowJointTrajectoryGoal( trajectory=self.jt, goal_time_tolerance=dur+rospy.Duration(2) )

	def send_command(self):
		self.client.wait_for_server()
		print self.goal
		self.client.send_goal(self.goal)

		self.client.wait_for_result()
		print self.client.get_result()				

class Contour:
	def __init__(self, cnt):
		self.contour = cnt
		leftmost = tuple(cnt[cnt[:,:,0].argmin()][0])
		rightmost = tuple(cnt[cnt[:,:,0].argmax()][0])
		topmost = tuple(cnt[cnt[:,:,1].argmin()][0])
		bottommost = tuple(cnt[cnt[:,:,1].argmax()][0])
#		self.bottomLeftCorner = tuple([leftmost[0], bottommost[1]])
#		self.topRightCorner = tuple([rightmost[0], topmost[1]])	
		self.topLeftCorner = tuple([leftmost[0], topmost[1]])
		self.bottomRightCorner = tuple([rightmost[0], bottommost[1]])

#	bottomLeftCorner = tuple([0,0])
#	topRightCorner = tuple([0,0])
	topLeftCorner = tuple([0,0])
	bottomRightCorner = tuple([0,0])

	contour = []
	def is_within(self, point):
		if point[0] > self.topLeftCorner[0] and point[0] < self.bottomRightCorner[0] and point[1] > self.topLeftCorner[1] and point[1] < self.bottomRightCorner[1]:
			return True
		else:
			return False

	def is_equal(self, cnt, tolerance = 0):
		if math.fabs(cnt.topLeftCorner[0]-self.topLeftCorner[0]) < tolerance and math.fabs(cnt.topLeftCorner[1]-self.topLeftCorner[1])<tolerance:
			return True
		else:
			return False   

def are_points_equal(point1, point2, tolerance):
	if math.fabs(point1[0]-point2[0]) < tolerance and math.fabs(point1[1]-point2[1]) < tolerance:
		return True
	else:
		return False
	
def generate_trajectorypoints(point1, point2, interval):
	trajectory_points = []
	if math.fabs(point1[0]-point2[0]) > 30: #virker i nu?
		slope = ((point2[1] - point1[1]) / (point2[0] - point1[0]))
		x = point1[0]
		if point1[0] > point2[0]: 
			point = []
			while x > point2[0]:
				x = x - interval
				y = y + x*slope
				z = 0.0
				point.append(x)
				point.append(y)
				point.append(z)
				trajectory_points.append(point)
		else:
			point = []
			while x < point2[0]:
				x = x + interval
				y = y + x*slope  
				z = 0.0
				point.append(x)
				point.append(y)
				point.append(z)
				trajectory_points.append(point)

	else:
		slope = ((point2[0] - point1[0]) / (point2[1] - point1[1]))
		y = point1[1]

		print "er vi her?"
		if point1[1] > point2[1]: 
			print "ja"
			point = []
			x = point2[0]
			while y > point2[1]:
				y = y - interval
				x = x + y*slope
				z = 0.0
				point.append(x)
				point.append(y)
				point.append(z)
				trajectory_points.append(point)
		else:
			point = []
			x = point1[0]
			while y < point2[1]:
				y = y + interval
				x = x + y*slope  
				z = 0.0
				point.append(x)
				point.append(y)
				point.append(z)
				trajectory_points.append(point)
	
	return trajectory_points
			

# Color definitions
lower_red = np.array([50,10,70])
upper_red = np.array([151,110,180])

lower_white = np.array([150,150,150])
upper_white = np.array([255,255,255])
    
# Main
print "loading from file"
image = cv2.imread('/home/ubuntu/catkin_ws/src/robot_eksamensprojekt/src/hus.jpg')
cv2.imshow('raw image',image)

mask_red = cv2.inRange(image, lower_red, upper_red)
mask_white = cv2.inRange(image, lower_white, upper_white)

res_red = cv2.bitwise_and(image,image, mask= mask_red)
res_white = cv2.bitwise_and(image,image, mask= mask_white)

edges_white = cv2.Canny(res_white,150,250)

contours_white,hierarchy = cv2.findContours(edges_white,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

papers = []
for cnt in contours_white:   
    if cv2.contourArea(cnt) > 40000:              	
		is_new = True
		cv2.drawContours(res_white,cnt,-1,(0,255,0),1)      
		c = Contour(cnt)		

		for p in papers:
			if c.is_equal(p,10):
				is_new = False

		if is_new:
			papers.append(c)
        
for p in papers:
    print cv2.contourArea(p.contour)

inside = []
for cnt in contours_white:
	c = Contour(cnt)
	if papers[0].is_within(c.topLeftCorner):
		is_new = True
		for i in inside:
			if c.is_equal(i,15):
				is_new = False

	if is_new:
		inside.append(c)     

points = []
for i in inside:
	cv2.drawContours(res_white,i.contour,-1,(0,0,255),1)
	points.append(cv2.approxPolyDP(i.contour, 3, False))

sorted_points = []
for p in points:
	for po in p:
		is_new = True
		for sp in sorted_points:
			if are_points_equal(po[0],sp,15):
				is_new = False

		if is_new:
			sorted_points.append(po[0])     

for sp in sorted_points:
	print sp

result = []
for p in sorted_points:
	x = p[0] - papers[0].topLeftCorner[0] + papers[1].topLeftCorner[0]
	y = p[1] - papers[0].topLeftCorner[1] + papers[1].topLeftCorner[1]
	result.append(tuple([x,y]))
	cv2.circle(res_white, (p[0], p[1]), 1, 0, 1)

for r in result:
	cv2.circle(res_white, r, 1, 0, 1)

point1_test = [90,148]
point2_test = [96,237]
tr_result = []
tr_result = generate_trajectorypoints(point1_test,point2_test,10)

print tr_result

print_result = []
for tr in tr_result:
	#print tr
	print_result.append(tuple([tr[0],tr[1]]))

for pr in print_result:
	cv2.circle(res_white, pr, 1, 0, 1)




cv2.imshow('res_white',res_white)

if __name__ == "__main__":
	rospy.init_node("invkin")

	node= ActionExampleNode("/arm_controller/follow_joint_trajectory")

	node.send_command()

key = cv2.waitKey(0)
if key == 27:
    exit(0)
