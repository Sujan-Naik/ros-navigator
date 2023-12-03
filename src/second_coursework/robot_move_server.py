#!/usr/bin/env python3
import rospy
import std_msgs
from actionlib_msgs.msg import GoalID
from second_coursework.srv import robot_move
from move_base_msgs.msg import MoveBaseActionGoal

rospy.init_node('main_node')
try:
    rospy.init_node("time")
except rospy.exceptions.ROSException as e:
    print("Node has already been initialized")

def move(request):
    """Publishes requests made to move the robot to the move_base goal"""
    pub = rospy.Publisher(name='/move_base/goal',data_class=MoveBaseActionGoal,queue_size=10)
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    goalID = GoalID()
    pub.publish(header,goalID,request)

def initialiseMovement():
    """Initialises the movement node and service"""
    rospy.init_node(name='robot_move_server', anonymous=True)
    rospy.Service(name='robot_move', service_class=robot_move, handler=move)
    rospy.spin()


if __name__ == '__main__':
    try:
        initialiseMovement()
    except rospy.ROSInterruptException:
        pass
