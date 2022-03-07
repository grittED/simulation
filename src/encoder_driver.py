#!/usr/bin/env python3

import sys
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Float64
from math import sin, cos

class sinusoidalControl1:
    def __init__(self):
        rospy.init_node("encoder_driver", anonymous=True)
        self.odom_pub = rospy.Publisher("odom_data_quat", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()

        x = 0
        y = 0
        th = 0
        lengthWheelBase = 3

        self.r = rospy.Rate(8.0)
        while not rospy.is_shutdown():
            right_speed = 3
            left_speed = 3
            lengthWheelBase = 3
    
            vx = (right_speed + left_speed)/2
            vy = 0
            vth = ((right_speed - left_speed)/lengthWheelBase)
    
            current_time = rospy.Time.now()
            last_time = rospy.Time.now()
    
            current_time = rospy.Time.now()
    
            # compute odometry in a typical way given the velocities of the robot
            dt = (current_time - last_time).to_sec()
            # dt=3
            delta_x = (vx * cos(th) - vy * sin(th)) * dt
            delta_y = (vx * sin(th) + vy * cos(th)) * dt
            delta_th = vth * dt
    
            x += delta_x
            y += delta_y
            th += delta_th
    
            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
    
            # # first, we'll publish the transform over tf
            self.odom_broadcaster.sendTransform(
                (x, y, 0.),
                odom_quat,
                current_time,
                "base_link",
                "odom"
            )
    
            # # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"
    
            # # set the position
            odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
    
            # # set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
    
            # # publish the message
            self.odom_pub.publish(odom)
    
            last_time = current_time
            self.r.sleep()


# call the class
def main(args):
    sc1 = sinusoidalControl1()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
