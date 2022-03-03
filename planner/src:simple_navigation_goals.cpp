 #include <ros/ros.h>
 #include <move_base_msgs/MoveBaseAction.h>
 #include <actionlib/client/simple_action_client.h>
 
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
 
 int main(int argc, char** argv){
   ros::init(argc, argv, "simple_navigation_goals");
 
   //tell the action client that we want to spin a thread by default
   MoveBaseClient ac("move_base", true);
 
   //wait for the action server to come up
   while(!ac.waitForServer(ros::Duration(5.0))){
     ROS_INFO("Waiting for the move_base action server to come up");
   }
 
   move_base_msgs::MoveBaseGoal goal;

   // The user will input four arguments to define the area that is to be gritted
   // For now, I have just chosen some values fot these
   recWidth = 5
   recLength = 10
   // We will also input which corner the robot is in and which way it is facing
   // For now we will assume the robot is in the bottom left hand corner and is facing down the length of the rectangle.

   // This planner will send the robot along the length of the rectangle before shifting along the width 0.5 meters and sending it back
   // Furthermore, the robot, when travelling the length, will do this in incriments of 1 meter. This is so that it can avoid obstacles but also grit the majority of the rectangle
   
   // First set counters for the length and width
   lengthCount = 0 
   widthCount = 0

   // We will also set a variable to store which direction the robot is currently facing
   yaw = 0

   while widthCount != recWidth{
       // First we grit the length of the rectabgle
        gritLength(recLength)       

       // Check which way robot is facing
       // TODO: If it is facing 'North' (original direction) turn right, else if it is facing 'South' turn left
       // Update 'yaw', which way the robot is facing
       // Move half a meter forwards (as below)
        goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.stamp = ros::Time::now();
 
        goal.target_pose.pose.position.x = 0.5;
        goal.target_pose.pose.orientation.w = 1.0;
 
        ROS_INFO("Sending goal");
        ac.sendGoal(goal);
 
        ac.waitForResult();
 
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Hooray, the base moved 1 meter forward");
        else
            ROS_INFO("The base failed to move forward 0.5 meters for some reason");
        return 0;

        // Increment counter
        widthCount += 0.5

        //TODO: Turn robot again (in same direction as before)
        // Update 'yaw', which way the robot is facing
   }
   // One more gritLength is required to grit the whole area
   gritLength(recLength)
   // By now, the entire rectangle is gritted and all we need to do is return the robot to its start location
   // (If only whole number of meters are entered) the robot should now be facing North in the top right corner
   // TODO Turn the robot 180 degrees
   // We call gritLength again to get the robot to travel to the bottom right coner
   gritLength(recLength)
   // TODO Turn the robot 90 degrees so it is facing "West"
   
   // Now we just have to travel recWidth back to the starting corner
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();
 
    goal.target_pose.pose.position.x = recWidth;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
 
    ac.waitForResult();
 
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved 1 meter forward");
    else
        ROS_INFO("The base failed to move forward recWidth meters for some reason");
      return 0;

    // Now the robot is in the bottom left hand corner,
    // TODO turn the robot 90 degrees so it is facing 'North' back in its original position
}

void gritLength(recLength) {
    lengthCount = 0
    while lengthCount != recLength{
            // Move robot one meter forward
            goal.target_pose.header.frame_id = "base_link";
            goal.target_pose.header.stamp = ros::Time::now();
 
             goal.target_pose.pose.position.x = 1.0;
            goal.target_pose.pose.orientation.w = 1.0;
 
            ROS_INFO("Sending goal");
            ac.sendGoal(goal);
 
            ac.waitForResult();
 
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("Hooray, the base moved 1 meter forward");
            else
                ROS_INFO("The base failed to move forward 1 meter for some reason");
            return 0;

            // Update count
            lengthCount += 1
       }
}

// TODO:
// Find out how inputs will be recieved
// Check if want to use base_link or map
// Find out how we want to turn. If we use map, do we need to turn? Or do we just put in coords, and the rest of the planner will sort out getting it there?
// Check what units we are going to allow. If we only allow whole numbers of meters for length/width, it simplifies problem and code above always work
// -> Could also use half meters, but gritLength function would have to be updated
// Check how we are going to return robot to the start, if using map, we could just input th start coords? The way I have done it above should work if we use 
//base_link, and allowing only whole number of meters to be entered
