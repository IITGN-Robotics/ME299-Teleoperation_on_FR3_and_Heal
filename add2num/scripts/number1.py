'''
This code creates a node called number_publisher_1 that publishes a number to the topic /number1 
'''
#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def publisher():
    rospy.init_node('number_publisher_1', anonymous=True)
   
    pub1 = rospy.Publisher('/number1', Int32, queue_size=10)
    #pub2 = rospy.Publisher('/number2', Int32, queue_size=10)
   
    rate = rospy.Rate(10)  # 10 Hz

    number1 = 12  # Example: first number to publish
    #number2 = 10  # Example: second number to publish

    while not rospy.is_shutdown():
        pub1.publish(number1)
        #pub2.publish(number2)
        #rospy.loginfo(f"Published: {number1} on /number1, {number2} on /number2")
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
