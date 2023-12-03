#!/usr/bin/env python3
import actionlib
import rospy
import std_msgs
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseGoal

import robot_move_server
from state_machines import state_machine
from second_coursework.msg import RoomCheckAction, RoomCheckGoal


try:
    rospy.init_node("time")
except rospy.exceptions.ROSException as e:
    print("Node has already been initialized")
goal = MoveBaseGoal()
goal.target_pose.header.stamp = rospy.get_rostime()
goal.target_pose.header.frame_id = 'map'
goal.target_pose.pose.position.x = 1.5
goal.target_pose.pose.position.y = 8.5
goal.target_pose.pose.position.z = 0
goal.target_pose.pose.orientation.x = 0
goal.target_pose.pose.orientation.y = 0
goal.target_pose.pose.orientation.z = 0
goal.target_pose.pose.orientation.w = 1
robot_move_server.move(goal)
state_machine.run()

client = actionlib.SimpleActionClient('/server', RoomCheckAction)
client.wait_for_server()
# header = std_msgs.msg.Header()
# header.stamp = rospy.Time.now()
# goalID = GoalID()
room_check_goal = RoomCheckGoal()
room_check_goal.times_to_check = 3
client.send_goal(room_check_goal)