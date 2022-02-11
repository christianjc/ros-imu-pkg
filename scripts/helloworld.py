#!/usr/bin/env python

import rospy

def helloworld():
    
    # Initialize node with a default name imu_node_python
    rospy.init_node("imu_node_python", anonymous=True)

    # Print to INFO log
    rospy.loginfo("Hello World")


if __name__== '__main__':
    try:
        helloworld()
    except rospy.ROSInterruptException:
        pass