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

def main():
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