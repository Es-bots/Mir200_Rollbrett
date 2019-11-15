#!/usr/bin/env python
import rospy
import actionlib
from actions_tutorial.msg import WashTheDishesAction, WashTheDishesFeedback, WashTheDishesResult

class ActionServer():
    def __init__(self):
        self.a_server = actionlib.SimpleActionServer(
            "wash_dishes_as",WashTheDishesAction,execute_cb=self.execute_cb , auto_start=False)

        self.a_server.start()


    def execute_cb(self, goal):
        success = True
        last_dishes_washed = ''
        feedback = WashTheDishesFeedback()
        result = WashTheDishesResult()
        rate = rospy.Rate(1)
        for i in range(0, goal.number_of_minutes):
            if self.a_server.is_preempt_requested():
                success = False
                break
            last_dishes_washed = 'bowl-'+str(i)
            feedback.last_dish_washed = last_dishes_washed
            result.dishes_washed.append(last_dishes_washed)
            self.a_server.publish_feedback(feedback)
            rate.sleep()
        if success:
            self.a_server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('action_server')
    ActionServer()
    rospy.spin()