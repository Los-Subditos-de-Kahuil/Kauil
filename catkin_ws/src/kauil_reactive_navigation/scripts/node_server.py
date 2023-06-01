#!/usr/bin/env python
import rospy
import sys
import os
import numpy as np
from reactive_nav.srv import *

def callback_function(req):
    goals = []
    with open(os.path.join(sys.path[0], "./../resources/goals.txt")) as f:
        for line in f.readlines():
            goals.append(np.fromstring(line, sep=",").tolist())

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
    rospy.init_node('node_server')
    s=rospy.Service('get_new_goal', getNewGoal, callback_function)
    rospy.spin()