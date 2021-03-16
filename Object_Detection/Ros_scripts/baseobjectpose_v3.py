#!/usr/bin/env python
import numpy as np
import math
import math3d as m3d
import rospy
import csv
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from std_msgs.msg import String , Float32MultiArray , Float32
from iiwa_msgs.msg import CartesianPose


#change this path to where you have the gripperpos file and the txt file
fgripperpos = '/home/mostafa/ComputerVision-Project/Object_Detection/Ros_script/gripperPos'
txt_file = '/home/mostafa/ComputerVision-Project/yolov5/bo7sen.txt'



current_pose = [0,0,0,0,0,0,0]

def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

def PublishingPose():
		#################### determining transformation matrix of Object w.r.t Camera ################################

	pub = rospy.Publisher('/desired_pose', PoseStamped, queue_size=100)
	# rospy.init_node('camera_pose', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		#indicate from where to read the values of X,Y,Z from the txt file
		text = open(txt_file, 'r')							#Yolo
		# text = open('/home/mostafa/ComputerVision-Project/yolov5/Hough Algorithim/bo7sen.txt', 'r')   #Hough Transform
		for line in text:
			t = line.split(" ")
			Xc,Yc,Zc = float(t[0]),float(t[1]),float(t[2])

		# text.close()

		T_co = np.eye(4)
		T_co[0,-1],T_co[1,-1],T_co[2,-1] = Xc,Yc,Zc



		#################### transformation Camera w.r.t Flange ################################
					# Hand-Eye ( from'handeyedata' file) #
		T_fc = np.array([
		[-0.999522,  -0.030504,  -0.005003, -11.167931],
		[0.030022,  - 0.996506,   0.077945,  52.951744],
		[-0.007364,   0.077757,   0.996945, -39.348851],
		[0,        0,		      0		  ,1		  ]
		])
		# T_fc = T_fc 


		#################### determining transformation matrix of Flange w.r.t Base  ################################

		current_position = [current_pose[0]*1000, current_pose[1]*1000, current_pose[2]*1000]
		current_orientation = [current_pose[6], current_pose[3], current_pose[4], current_pose[5]]
		T_bf = np.eye(4)
		T_bf[0:3,0:3] = quaternion_rotation_matrix(current_orientation)
		T_bf[0,-1],T_bf[1,-1],T_bf[2,-1] = current_position

		#################### transformation End-effector w.r.t Flange ################################
		#################### Might be End-effector w.r.t Flange ################################ 		!!!!!

		grippercamera_pos = np.loadtxt(fgripperpos)
		T_fe = np.eye(4)
		T_fe[0,-1],T_fe[1,-1],T_fe[2,-1] = grippercamera_pos[:3]

		#################### calculating transformation matrix of Object w.r.t Base link ################################

		# T_bo = np.dot(np.dot(T_bf , np.linalg.inv(T_fc)) , T_co)
		T_bo = np.dot(np.dot(T_bf , T_fc) , T_co)

		#################### calculating transformation matrix of End-effector w.r.t Base link ################################

		T_be = np.dot(T_bf , T_fe)

		pose_goal = PoseStamped() 				#Pose()

		pose_goal.pose.position.x = (T_bo[0,-1] - T_fe[0,-1]) /1000
		pose_goal.pose.position.y = (T_bo[1,-1] - T_fe[1,-1]) /1000
		pose_goal.pose.position.z = (T_bo[2,-1] - T_fe[2,-1]) /1000
		# pose_goal.pose.position.x = T_bo[0,-1]
		# pose_goal.pose.position.y = T_bo[1,-1]
		# pose_goal.pose.position.z = T_bo[2,-1]
		print(pose_goal)	
		pub.publish(pose_goal)

			
def pos_callback(data):

	global current_pose	
	current_pose[0] = data.poseStamped.pose.position.x
	current_pose[1] = data.poseStamped.pose.position.y
	current_pose[2] = data.poseStamped.pose.position.z
	current_pose[3] = data.poseStamped.pose.orientation.x
	current_pose[4] = data.poseStamped.pose.orientation.y
	current_pose[5] = data.poseStamped.pose.orientation.z
	current_pose[6] = data.poseStamped.pose.orientation.w
	# print(current_pose)

if __name__ == '__main__':
    try:

        rospy.init_node('callback', anonymous=True)
        data = rospy.Subscriber("/iiwa/state/CartesianPose", CartesianPose, pos_callback, queue_size =10)    

        PublishingPose()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

        ############### If needed to publish once ###############
    # try:
    #     m = PublishingPose()
    #     while not rospy.is_shutdown():
    #         connections = m.pub.get_num_connections()
    #         rospy.loginfo('Connections: %d', connections)
    #         if connections :> 0 :
    #             m.run()
    #             rospy.loginfo('Published')
    #             break
    #         rate.sleep()
    # except rospy.ROSInterruptException:

    #     pass
	# x1 =99.14 y1 -544.21  x1 = 112.6 y2 -554.43  x2-x1 = 13.46, y2-y1 = -10.22
