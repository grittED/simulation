#!/usr/bin/env python
# license removed for brevity

import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client(double x, double y):

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
   # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        result = movebase_client(0, 0)
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

    # The user will input four arguments to define the area that is to be gritted
    # For now, I have just chosen some values fot these
    recWidth = 5
    recLength = 10
    # We will also input which corner the robot is in and which way it is facing
    # For now we will assume the robot is in the bottom left hand corner and is facing down the length of the rectangle.

    # This planner will send the robot along the length of the rectangle before shifting along the width 0.5 meters and sending it back
    # Furthermore, the robot, when travelling the length, will do this in incriments of 1 meter. This is so that it can avoid obstacles but also grit the majority of the rectangle
   
    # First set counters for the length and width
    lengthCount = 0 
    widthCount = 0

    while widthCount != recWidth:
        # Move the robot along the length once
        while lengthCount != recLength:
            lengthCount += 0.5
            result = move_base_client(widthCount, lengthCount)
        # Shift the robot across the width by 0.5 
        widthCount += 0.5:
        result = move_base_client(widthCount, lengthCount)
        # Move the robot back down the length
        while lengthCount != 0:
            lengthCount -= 0.5
            result = move_base_client(widthCount, lengthCount)
        # Shift the robot along the width another 0.5m
        widthCount += 0.5
        result = move_base_client(widthCount, lengthCount)

    # Then we need to do one final run to grit the final length of the rectangle
    while lengthCount != recLength:
            lengthCount += 0.5
            result = move_base_client(widthCount, lengthCount)
    
    # Finally we return the robot to the starting poiny
    result = move_base_client(0,0)

    # TODO Do we need to define the orientation of the robot?
