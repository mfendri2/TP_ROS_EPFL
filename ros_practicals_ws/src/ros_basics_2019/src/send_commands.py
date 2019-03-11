#!/usr/bin/env python

import rospy 
from geometry_msgs.msg import Point,Pose

def talker():

	command=rospy.Publisher('goal_pos',Point,queue_size=10) 
	rospy.init_node('talker',anonymous=True)
	rate = rospy.Rate(10)

	while(True):
		point = Point()
		point.x = float(input('Send point x: '))
		point.y = float(input('Send point y: '))
		point.z = 0
		command.publish(point)
		print('sent')
		rate.sleep()
	
	rospy.spin() 



if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
