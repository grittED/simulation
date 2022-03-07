#!/usr/bin/env python3

import sys
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class vel_converter:
    def __init__(self):
        rospy.init_node("vel_converter", anonymous=True)
        self.cmdVelSub = rospy.Subscriber("cmd_vel", Twist, self.callback)
        self.rightPub = rospy.Publisher("/setpoint/left", Float64, queue_size=10)
        self.leftPub = rospy.Publisher("/setpoint/right", Float64, queue_size=10)

        self.WHEEL_BASE = 4
        self.WHEEL_RADIUS = 3

        self.left_vel = Float64()
        self.right_vel = Float64()

    def callback(self, data):
        linear_velocity = data.linear.x
        angular_velocity = data.angular.z
        self.left_vel = (linear_velocity - angular_velocity * self.WHEEL_BASE / 2.0)/self.WHEEL_RADIUS
        self.right_vel = (linear_velocity + angular_velocity * self.WHEEL_BASE / 2.0)/self.WHEEL_RADIUS
        self.publish_setpoints()

    def publish_setpoints(self):
        # Publish the results
        self.rightPub.publish(self.right_vel)
        self.leftPub.publish(self.left_vel)


# call the class
def main(args):
    vc = vel_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
