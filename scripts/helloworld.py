#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import random

def helloworld():
    
    # Initialize node with a default name imu_node_python
    rospy.init_node("imu_node_python", anonymous=True)

    global pub_data, data

    pub_data = rospy.Publisher('sensor_msg_py', Imu, queue_size=10)

    data = Imu()

    loop_rate = 2

    rate = rospy.Rate(loop_rate)


    # Print to INFO log
    rospy.loginfo("Starting node")

    while not rospy.is_shutdown():
        
        data.angular_velocity.x +=1

        pub_data.publish(data)

        rate.sleep()


if __name__== '__main__':
    try:
        helloworld()
    except rospy.ROSInterruptException:
        pass