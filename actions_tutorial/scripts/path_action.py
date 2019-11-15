#!/usr/bin/env python
import rospy
import actionlib
from mbf_msgs.msg import ExePathAction , ExePathGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
################################################## frange function
def frange(start, stop=None, step=None):

    if stop == None:
        stop = start + 0.0
        start = 0.0

    if step == None:
        step = 1.0

    while True:
        if step > 0 and start >= stop:
            break
        elif step < 0 and start <= stop:
            break
        yield ("%g" % start) # return float number
        start = start + step
################################################## call server Function
def call_server():
    client = actionlib.SimpleActionClient('/move_base_flex/exe_path', ExePathAction)
    client.wait_for_server()
    my_goal = ExePathGoal()
    my_path = Path()
    my_path.header.frame_id = 'map' 
    my_path.header.stamp = rospy.Time.now()
    for i in frange(0,3,0.2):
        my_posestamped = PoseStamped()
        my_posestamped.header.stamp = rospy.Time.now()
        my_posestamped.header.frame_id = 'map'
        my_posestamped.pose.position.x = float(i)
        my_posestamped.pose.position.y = -float(i)
        my_posestamped.pose.orientation.w = 1.0
        my_path.poses.append(my_posestamped)
        
    rospy.loginfo(my_path)
    my_goal.controller = 'DWAPlannerROS'
    my_goal.path = my_path
    client.send_goal(my_goal)
    client.wait_for_result()
    result = client.get_result()
    return result


if __name__ == '__main__':
    try:
        rospy.init_node('path_action')
        result=call_server()
        print 'The result is:' , result
    except rospy.ROSInterruptException as e:
        print "Something went wrong:" , e