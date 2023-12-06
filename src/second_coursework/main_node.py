#!/usr/bin/env python3
import actionlib
import rospy
import std_msgs
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseGoal
from std_msgs.msg import String

import robot_move_server
import yolo
from state_machines import state_machine
from second_coursework.msg import RoomCheckAction, RoomCheckGoal
from second_coursework.srv import robot_move

try:
    rospy.init_node("time")
except rospy.exceptions.ROSException as e:
    print("Node has already been initialized")

robot_move_server.robot_move_server()
yolo.start()
state_machine.run()

client = actionlib.SimpleActionClient('server', RoomCheckAction)
client.wait_for_server()
room_check_goal = RoomCheckGoal()
room_check_goal.times_to_check = 3
client.send_goal(room_check_goal)