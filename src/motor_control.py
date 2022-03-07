#!/usr/bin/env python3

import sys
import rospy
from motors import Motors
from std_msgs.msg import Float64

class motor_control:
    def __init__(self):
        rospy.init_node("motor_control", anonymous=True)
        self.leftSub = rospy.Subscriber("control/left", Float64, self.callbackLeft)
        self.rightSub = rospy.Subscriber("control/right", Float64, self.callbackRight)

        # Initialise motor control
        self.mc = Motors()
        # Define where motors are plugged into shield
        self.motor_left_id_0 = 2
        self.motor_left_id_1 = 3
        self.motor_right_id_0 = 4
        self.motor_right_id_1 = 5
        # Set Robot move speed
        speed = 100

    def callbackLeft(self, data):
        self.move_left(int(data.data))

    def callbackRight(self, data):
        self.move_right(int(data.data))


    def move_left(self, input_speed):
        self.mc.move_motor(self.motor_left_id_0, input_speed)
        self.mc.move_motor(self.motor_left_id_1, input_speed)

    def move_right(self, input_speed):
        self.mc.move_motor(self.motor_right_id_0, input_speed)
        self.mc.move_motor(self.motor_right_id_1, input_speed)


# call the class
def main(args):
    mc = motor_control()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
