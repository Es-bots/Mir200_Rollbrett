#!/usr/bin/env python
import rospy
import actionlib
from mbf_msgs.msg import ExePathAction , ExePathGoal , GetPathAction , GetPathGoal , GetPathActionResult
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
################################################################# calling goal position back by subscriber and execute action clinet
def goal_cb(goal_msg):
    x_goal=goal_msg.pose.position.x
    y_goal=goal_msg.pose.position.y
    z_goal=goal_msg.pose.orientation.z
    w_goal=goal_msg.pose.orientation.w
    print("x: "+str(x_goal)+" y: "+str(y_goal)+" z: "+ str(z_goal)+" w: "+str(w_goal))
    client = actionlib.SimpleActionClient('/move_base_flex/get_path', GetPathAction)
    client.wait_for_server()
    my_goal = GetPathGoal()
    my_goal.use_start_pose = False
    my_goal.target_pose.header.stamp = rospy.Time.now()
    my_goal.target_pose.header.frame_id = "map"
    my_goal.target_pose.pose.position.x = x_goal
    my_goal.target_pose.pose.position.y = y_goal
    my_goal.target_pose.pose.orientation.z = z_goal
    my_goal.target_pose.pose.orientation.w = w_goal
    my_goal.tolerance = 0.0
    my_goal.planner = "SBPLLatticePlanner"
    print("sending goal")
    client.send_goal(my_goal)
    client.wait_for_result()
################################################################# Subscribe to /move_base_flex/get_path/result and Action Client to Exe_path
def path_cb(path_msg):
    robot_path = path_msg.result.path.poses
    j=0
    x_robot_plan = []
    y_robot_plan = []
    z_robot_plan = []
    w_robot_plan = []
    for i in robot_path:
        x_robot_plan.append(round(robot_path[j].pose.position.x,3))
        y_robot_plan.append(round(robot_path[j].pose.position.y,3))
        z_robot_plan.append(round(robot_path[j].pose.orientation.z,3))
        w_robot_plan.append(round(robot_path[j].pose.orientation.w,3))
        j+=1
    client = actionlib.SimpleActionClient('/move_base_flex/exe_path', ExePathAction)
    client.wait_for_server()
    exe_path_goal = ExePathGoal()
    my_path = Path()
    my_path.header.frame_id = 'map' 
    my_path.header.stamp = rospy.Time.now()
    k = 0
    for i in x_robot_plan:
        my_posestamped = PoseStamped()
        my_posestamped.header.stamp = rospy.Time.now()
        my_posestamped.header.frame_id = 'map'
        my_posestamped.pose.position.x = i
        my_posestamped.pose.position.y = y_robot_plan[k]
        my_posestamped.pose.orientation.z = z_robot_plan[k]
        my_posestamped.pose.orientation.w = w_robot_plan[k]
        my_path.poses.append(my_posestamped)
        k+=1
    exe_path_goal.controller = 'DWAPlannerROS'
    exe_path_goal.path = my_path
    client.send_goal(exe_path_goal)
    client.wait_for_result()
################################################################# Main
if __name__ == '__main__':
    try:
        rospy.init_node('goal_to_motion')
        goal_sub = rospy.Subscriber("/move_base_simple/goal",PoseStamped,goal_cb)
        #path_sub = rospy.Subscriber("/move_base_flex/get_path/result",GetPathActionResult,path_cb)
        #print 'The result is:' , result
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print "Something went wrong:" , e