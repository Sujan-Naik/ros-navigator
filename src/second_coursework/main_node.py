#!/usr/bin/env python3
import actionlib
import rospy

import robot_move_server, yolo, state_machine
from second_coursework.msg import RoomCheckAction, RoomCheckGoal


def main():
    try:
        rospy.init_node("time")
    except rospy.exceptions.ROSException:
        print("Node has already been initialized")

    try:
        rospy.init_node("main")
    except rospy.exceptions.ROSException:
        print("Node has already been initialized")

    client = actionlib.SimpleActionClient('robot_state_machine_action_server', RoomCheckAction)
    client.wait_for_server()
    room_check_goal = RoomCheckGoal()
    room_check_goal.times_to_check = 1
    client.send_goal(room_check_goal)

    client.wait_for_result()
    result = client.get_result()

    rule_breaks = result.rule_break_amount
    print(f'\nRule 1 was broken {rule_breaks[0]} times')
    print(f'\nRule 2 was broken {rule_breaks[1]} times')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass