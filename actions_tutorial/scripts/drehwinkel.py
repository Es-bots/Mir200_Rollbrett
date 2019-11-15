#!/usr/bin/env python
import rospy
import math
from mbf_msgs.msg import GetPathAction, GetPathGoal, GetPathGoal, GetPathActionResult
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

############################################################ Global Variables
robot_pos_x = 0.0
robot_pos_y = 0.0
robot_ori_z = 0.0
robot_ori_w = 0.0
robot_vel_x = 0.0
robot_vel_z = 0.0
goal_x = 0.0
goal_y = 0.0
############################################################ calling Robot actual Position(x,y,z,w) and Velocity(x,z)
def callback_act_pose(pos_msg):
    global robot_pos_x,robot_pos_y,robot_ori_z,robot_ori_w,robot_vel_x,robot_vel_z
    robot_pos_x=round(pos_msg.pose.pose.position.x,4)
    robot_pos_y=round(pos_msg.pose.pose.position.y,4)
    robot_ori_z=round(pos_msg.pose.pose.orientation.z,4)
    robot_ori_w=round(pos_msg.pose.pose.orientation.w,4)
    robot_vel_x=round(pos_msg.twist.twist.linear.x,4)
    robot_vel_z=round(pos_msg.twist.twist.angular.z,4)
###########################################################
def callback_goal(goal_msg):
    global goal_x, goal_y
    goal_x = goal_msg.pose.position.x
    goal_y = goal_msg.pose.position.y
    rospy.loginfo("Goal Position: "+"("+str(goal_x)+","+str(goal_y)+")")
############################################################ calling published global path and determine the Turning points
def path_cb(path_msg):
    global goal_x, goal_y,robot_pos_x,robot_pos_y,robot_ori_z,robot_ori_w,robot_vel_x,robot_vel_z
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
#--------------------------------------------------------------------------------------------
    k = 0
    l = 0
    turn_x=[]
    turn_y=[]
    length_z=len(z_robot_plan)
    initial_x = round(robot_pos_x,3)
    initial_y = round(robot_pos_y,3)
    initial_angle = round(robot_ori_z,3)
    #rospy.loginfo(z_robot_plan)
    #rospy.loginfo("Initial angle= "+ str(initial_angle))
    for i in z_robot_plan:
        angle = abs(initial_angle) - i
        #rospy.loginfo("Diff Angle= "+str(angle))
        k+=1
        if (angle > 0.17 or angle< -0.17) :
            rospy.loginfo("turning recognised at Array["+str(k-1)+"]"+" in position "+"("+str(x_robot_plan[k-1])+","+str(y_robot_plan[k-1])+")")
            if len(turn_x)>=1: # if there is at least one element in the Turn List
                if ((x_robot_plan[k-1] - turn_x[l]) > 2.0) or ((x_robot_plan[k-1] - turn_x[l]) < -2.0): # if distance between the (new) recognized point and previous point greater than 2.0m is
                    rospy.loginfo("distance between the new recognized point and previous point is greater than 2.0m")
                    if abs(math.sqrt(((x_robot_plan[k-1]-goal_x)**2) +((y_robot_plan[k-1]-goal_y)**2))) > 2 : # if distance between the new recognized point and the goal point greater than 2m is:
                        rospy.loginfo("Distance between the new recognized point and the goal point is greater than 2m")
                        turn_x.append(x_robot_plan[k-1])
                        turn_y.append(y_robot_plan[k-1])
                        l+=1
                    else:
                        rospy.loginfo("Distance between the new recognized point and the goal point isn't greater than 2m")
                else:
                    rospy.loginfo("distance between the new recognized point and previous point isn't greater than 2.0m")
            elif len(turn_x)==0: # if List turn_x is empty, add the first Point, if and only if this point is 2 Meter farther than the Initial point
                if abs(math.sqrt(((x_robot_plan[k-1]-initial_x)**2) +((y_robot_plan[k-1]-initial_y)**2))) > 2 : # if distance between the recognized Point and initial Point greater as 2 Meter is, add this Point ti the List.
                    rospy.loginfo("The recognised Turn is enough farther than initial Point")
                    turn_x.append(x_robot_plan[k-1])
                    turn_y.append(y_robot_plan[k-1])
                else:
                    rospy.loginfo("The recognised Turn is too close to initial Point")
            initial_angle= i
        if k==length_z:
            if len(turn_x) > 1:
                j = -1 
                for itrt in turn_x:
                    j+=1
                    dist = abs(math.sqrt(((turn_x[j+1]-turn_x[j])**2) +((turn_y[j+1]-turn_y[j])**2)))
                    print("Distance between point("+str(turn_x[j+1])+","+str(turn_y[j+1])+")"+" and"+" ("+str(turn_x[j])+","+str(turn_y[j])+")"+"is: "+str(dist))
                    if dist < 3:
                        rospy.loginfo("Distance between two point in turn_x array is less than 3m, the SECOND one will be deleted!")
                        turn_x.pop(j+1)
                        turn_y.pop(j+1)
                        j-=1
                        if len(turn_x)==1:
                            break
                    if (j+2) == len(turn_x):
                        break    
            print("Turning x-Areas are= "+str(turn_x))
            print("Turning y-Areas are= "+str(turn_y))
            print("Break")
            z_robot_plan=[]
            break




if __name__ == '__main__':
    rospy.init_node('drehwinkel', anonymous=False)
    rospy.loginfo("node initialized...")
    sub_odom = rospy.Subscriber("/odometry/filtered",Odometry,callback_act_pose)
    sub_goal = rospy.Subscriber("/move_base_simple/goal",PoseStamped,callback_goal)
    path_sub = rospy.Subscriber("/move_base_flex/get_path/result",GetPathActionResult,path_cb)
    rospy.spin()