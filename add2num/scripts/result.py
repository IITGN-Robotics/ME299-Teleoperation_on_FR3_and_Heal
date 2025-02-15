#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

class AddTwoNumbers:
    def __init__(self):
        rospy.init_node('number_adder', anonymous=True)

        self.num1 = 0
        self.num2 = 0

        rospy.Subscriber('/number1', Int32, self.callback1)
        rospy.Subscriber('/number2', Int32, self.callback2)

        self.result_pub = rospy.Publisher('/result', Int32, queue_size=10)
        self.rate = rospy.Rate(1)  # 10 Hz

        self.run()

    def callback1(self, data):
        self.num1 = data.data
        #rospy.loginfo(f"Received /number1: {self.num1}")

    def callback2(self, data):
        self.num2 = data.data
        #rospy.loginfo(f"Received /number2: {self.num2}")

    def run(self):
        while not rospy.is_shutdown():
            result = self.num1 + self.num2
            self.result_pub.publish(result)
            #rospy.loginfo(f"Published /result: {result}")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        AddTwoNumbers()
    except rospy.ROSInterruptException:
        pass