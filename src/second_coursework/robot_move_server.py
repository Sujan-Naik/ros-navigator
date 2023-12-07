#!/usr/bin/env python3
import random

import actionlib
import rospy
import std_msgs
from actionlib_msgs.msg import GoalID
from second_coursework.srv import robot_move
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal, MoveBaseAction, MoveBaseActionResult


"Taken from decompiled itr_cw_video.pyc"
ROOM_A_CORNERS = [
 {'x':0.1178496852517128,
  'y':10.650749206542969},
 {'x':3.76343035697937,
  'y':10.621451377868652},
 {'x':0.1178496852517128,
  'y':6.036409378051758},
 {'x':3.7487902641296387,
  'y':6.051058292388916}]
ROOM_B_CORNERS = [{'x':3.805814266204834,  'y':10.671765327453613},
 {'x':8.269617080688477,
  'y':10.644081115722656},
 {'x':3.805814266204834,
  'y':6.04871129989624},
 {'x':8.278841018676758,
  'y':6.021029472351074}]
ROOM_D_CORNERS = [{'x':0.10946185886859894,  'y':5.901395320892334},
 {'x':3.712092161178589,
  'y':5.8719706535339355},
 {'x':0.10946184396743774,
  'y':0.0973321795463562},
 {'x':3.667977809906006,
  'y':0.0973321795463562}]

def get_random_point(corners, axis):
    lowest = 100
    highest = 0
    for point in corners:
        value = point[axis]
        if value > highest:
            highest = value
        if value < lowest:
            lowest = value
    return random.uniform(lowest,highest)


def robot_move_callback(request):
    """Publishes requests made to move the robot to the move_base goal"""

    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.get_rostime()
    goal.target_pose.header.frame_id = 'map'

    room_name = request.room_name

    if room_name == 'A':
        goal.target_pose.pose.position.x = get_random_point(ROOM_A_CORNERS, 'x')
        goal.target_pose.pose.position.y = get_random_point(ROOM_A_CORNERS, 'y')

    elif room_name == 'B':
        goal.target_pose.pose.position.x = get_random_point(ROOM_B_CORNERS, 'x')
        goal.target_pose.pose.position.y = get_random_point(ROOM_B_CORNERS, 'y')

    elif room_name == 'D':
        goal.target_pose.pose.position.x = get_random_point(ROOM_D_CORNERS, 'x')
        goal.target_pose.pose.position.y = get_random_point(ROOM_D_CORNERS, 'y')
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
    client.wait_for_result(rospy.Duration.from_sec(30))
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
