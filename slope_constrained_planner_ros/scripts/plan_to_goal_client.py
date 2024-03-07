#!/usr/bin/env python3
"""
.. module:: plan_to_goal_client
    :platform: Unix
    :synopsis: the plan_to_goal_client python script in slope_constrained_planner package

Subscribes to:
    /goal

Uses Action:
    /slope_constrained_planner/plan_to_goal


A helper planner client node which waits for the target point to be set, and then sends it to the 
planner action server.
"""
import rospy
from geometry_msgs.msg import PoseStamped
import actionlib
import slope_constrained_planner_msgs.msg



class PathToGoalClient:
    """
    Planner client class which defines the subcription and action client info. Additionally,
    it has a member function for goal callback to send the goal to planner action server
    """

    def goalCallback(self, msg):
        """
            Callback function for ``/goal`` topic subscription. Waits for the planner action server 
            to be available and sends the goal point.
        """
        goal = slope_constrained_planner_msgs.msg.PlanToGoalGoal()
        goal.goal = msg
        self.client.wait_for_server()
        self.client.send_goal(goal)

    def __init__(self):
        """
            Defines the subscription to ``/goal`` topic and planner action server client node.
        """
        self.sub = rospy.Subscriber('/goal', PoseStamped, self.goalCallback)

        self.client = actionlib.SimpleActionClient('/slope_constrained_planner/plan_to_goal', slope_constrained_planner_msgs.msg.PlanToGoalAction)

        print('Waiting for plan_to_goal server to appear...')
        self.client.wait_for_server()
        print('Found server.')



if __name__ == '__main__':
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('path_to_goal_client')
    client = PathToGoalClient()
    rospy.spin()
