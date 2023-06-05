#!/usr/bin/env python
"""
This service receives the currently reached goal and returns the next goal to reach.

Authors:
    - Alejandro Dominguez Lugo      (A01378028)
    - Diego Alberto Anaya Marquez   (A01379375)
    - Nancy Lesly Garcia Jimenez    (A01378043)

Date: 2023/05/25
"""
import rospy
import sys
import os
import numpy as np
from reactive_nav.srv import *


def callback_function(req):
    """Given the currently reached goal, returns the next goal to reach.

    Args:
        req (getNewGoalRequest): Request containing the currently reached goal.

    Returns:
        getNewGoalResponse: Response containing the next goal to reach.
    """
    # * Read goals from file
    goals = []
    with open(os.path.join(sys.path[0], "./../resources/goals.txt")) as f:
        for line in f.readlines():
            goals.append(np.fromstring(line, sep=",").tolist())

    # * Get new goal
    old_goal = [req.x, req.y]
    i = 0
    new_goal = None
    for goal in goals:
        if goal == old_goal:
            if i == len(goals) - 1:
                new_goal = goals[0]
            else:
                new_goal = goals[i + 1]
        i += 1

    return getNewGoalResponse(new_goal[0], new_goal[1])


if __name__ == "__main__":
    try:
        rospy.init_node("node_server")
        s = rospy.Service("get_new_goal", getNewGoal, callback_function)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
