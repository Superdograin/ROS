import time
import rospy
from geometry_msgs.msg import Twist
'''
import Car

car = Car.Car()


linear[x,y,z]	angular[z]
前进	( 1, 0, 0, 0)	lx
后退	(-1, 0, 0, 0)	lx
左平移	( 0, 1, 0, 0)	ly
右平移	( 0,-1, 0, 0)	ly

左转	( 0, 0, 0, 1)	az
支点左转( 0, 1, 0, 1)	az
中心左转( 0,-1, 0, 1)	az

左前	( 1, 0, 0, 1)	az
右后	(-1, 0, 0, 1)	az

右转	( 0, 0, 0,-1)	az
支点右转( 0, 1, 0,-1)	az
中心右转( 0,-1, 0,-1)	az

右前	( 1, 0, 0,-1)	az
左后	(-1, 0, 0,-1)	az


def Callback(move):
	if move.angular.z==0:#(x,x,x,0)
		if move.linear.x>0: #前进(1,x,x,0)
			rospy.loginfo("go forword!")
			if (move.linear.x*100.0>255.0):
				car.forward(255, 0.1)
			else:
				car.forward(int(move.linear.x*100.0), 0.1)
		elif move.linear.x<0: #后退(-1,x,x,0)
			rospy.loginfo("go back!")
			if (move.linear.x*100.0<-255.0):
				car.backward(255,0.1)
			else:
				car.backward(int(-move.linear.x*100.0),0.1)
		else :	#(0,x,x,0)
			if move.linear.y>0:#左平移(0,1,x,0)
				rospy.loginfo("pan left!")
				if (move.linear.y*100.0>255.0):
					car.pan_left(255,0.1)
				else:
					car.pan_left(int(move.linear.y*100.0),0.1)
			elif move.linear.y<0:#右平移(0,-1,x,0)
				rospy.loginfo("pan right!")
				if (move.linear.y*100.0<-255.0):
					car.pan_right(255,0.1)
				else:
					car.pan_right(int(-move.linear.y*100.0),0.1)

	elif move.angular.z>0:#(x,x,x,1)
		if move.linear.x==0:#(0,x,x,1)
			if move.angular.y==0 :#左转(0,0,x,1)
				rospy.loginfo("left!")
				if (move.angular.z*100.0>255.0):
					car.pinwheel_counterclockwise(255,0.1)
				else:
					car.pinwheel_counterclockwise(int(move.angular.z*100.0),0.1)
			elif move.linear.y>0 :#支点左转(0,1,x,1)
				rospy.loginfo("Pivot left!")
				if (move.angular.z*100.0>255.0):
					car.left_mode_1(255,0.1)
				else:
					car.left_mode_1(int(move.angular.z*100.0),0.1)
			else :#中心左转(0,-1,x,1)
				rospy.loginfo("Center left!")
				if (move.angular.z*100.0>255.0):
					car.left_mode_2(255,0.1)
				else:
					car.left_mode_2(int(move.angular.z*100.0),0.1)

		elif move.linear.x>0:#左前(1,x,x,1)
			rospy.loginfo("left forward!")
			if (move.angular.z*100.0>255.0):
				car.left_forward(255,0.1)
			else:
				car.left_forward(int(move.angular.z*100.0),0.1)
		else :#右后(-1,x,x,1)
			rospy.loginfo("right backward!")
			if (move.angular.z*100.0>255.0):
				car.right_backward(255,0.1)
			else:
				car.right_backward(int(move.angular.z*100.0),0.1)

	elif move.angular.z<0:#(0,x,x,-1)
		if move.linear.x==0:
			if move.linear.y==0 :#右转(0,0,x,-1)
				rospy.loginfo("right!")
				if (move.angular.z*100.0<-255.0):
					car.pinwheel_clockwise(255,0.1)
				else:
					car.pinwheel_clockwise(int(-move.angular.z*100.0),0.1)
			elif move.linear.y>0 :#支点右转(0,1,x,-1)
				rospy.loginfo("Pivot right!")
				if (move.angular.z*100.0<-255.0):
					car.right_mode_1(255,0.1)
				else:
					car.right_mode_1(int(-move.angular.z*100.0),0.1)
			else :#中心右转(0,-1,x,-1)
				rospy.loginfo("Center left!")
				if (move.angular.z*100.0<-255.0):
					car.right_mode_2(255,0.1)
				else:
					car.right_mode_2(int(-move.angular.z*100.0),0.1)

		elif move.linear.x>0 :#右前(1,x,x,-1)
			rospy.loginfo("right forward!")
			if (move.angular.z*100.0<-255.0):
				car.right_forward(255,0.1)
			else:
				car.right_forward(int(-move.angular.z*100.0),0.1)
		else :#左后(-1,x,x,-1)
			rospy.loginfo("left backward!")
			if (move.angular.z*100.0<-255.0):
				car.left_backward(255,0.1)
			else:
				car.left_backward(int(-move.angular.z*100.0),0.1)
'''


def Callback_test(move):
	print("lx: " +str( move.linear.x) + "ly: " + str(move.linear.y) + "az: " +str(move.angular.z) + "\n")


def Vel_subscribe():
	rospy.init_node('Mycar', anonymous=True)
	rospy.Subscriber("cmd_vel", Twist, Callback_test, queue_size=1)
        #rospy.Subscriber("cmd_vel", Twist, Callback, queue_size=1)
	rospy.spin()


if __name__ == '__main__':
    Vel_subscribe()


