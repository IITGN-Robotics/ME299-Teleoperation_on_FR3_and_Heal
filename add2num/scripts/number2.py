#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def publisher():
    rospy.init_node('number_publisher_2', anonymous=True)
    pub2 = rospy.Publisher('/number2', Int32, queue_size=10)
   
    rate = rospy.Rate(10)  # 10 Hz
    number2 = 80  # Example: second number to publish

    while not rospy.is_shutdown():
        pub2.publish(number2)
        #rospy.loginfo(f"Published: {number2} on /number2")
        rate.sleep()
if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass