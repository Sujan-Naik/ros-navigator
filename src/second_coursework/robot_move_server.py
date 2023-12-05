#!/usr/bin/env python3
import actionlib
import rospy
import std_msgs
from actionlib_msgs.msg import GoalID
from second_coursework.srv import robot_move
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal, MoveBaseAction, MoveBaseActionResult

'''
Entrances for Rooms
A 1.9, 8.3
B 6, 8.4
C  10.7, 8.3
D 2, 3.3
E 6.1, 3.5
F 10.6, 2.5
'''


def robot_move_callback(request):
    """Publishes requests made to move the robot to the move_base goal"""

    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.get_rostime()
    goal.target_pose.header.frame_id = 'map'

    room_name = request.room_name

    if room_name == 'A':
        goal.target_pose.pose.position.x = 1.9
        goal.target_pose.pose.position.y = 8.3

    elif room_name == 'B':
        goal.target_pose.pose.position.x = 6
        goal.target_pose.pose.position.y = 8.4

    elif room_name == 'D':
        goal.target_pose.pose.position.x = 2
        goal.target_pose.pose.position.y = 3.3
    else:
        raise ValueError('Must either be A, B or D')

    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation.x = 0
    goal.target_pose.pose.orientation.y = 0
    goal.target_pose.pose.orientation.z = 0
    goal.target_pose.pose.orientation.w = 1

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(20))
    if client:
        return True

    return False


def robot_move_server():
    """Initialises the movement node and service"""
    try:
        rospy.init_node('robot_move_server')
    except rospy.exceptions.ROSException as e:
        print("Node has already been initialized")
    rospy.Service(name='robot_move_service', service_class=robot_move, handler=robot_move_callback)
