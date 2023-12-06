#!/usr/bin/env python3
import math

import actionlib
import actionlib_msgs.msg
import move_base_msgs.msg
import rospy
import smach
import std_msgs
from actionlib_msgs.msg import GoalID, GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from smach import StateMachine, Iterator, Sequence, CBState, Concurrence
from smach_ros import SimpleActionState, ActionServerWrapper
from second_coursework.msg import RoomCheckAction, RoomCheckActionGoal
from second_coursework.srv import robot_move, yolo_detect



# Rule 1: No people should be in the kitchen (Room D) at any time
# Rule 2: Cat and dog shouldn't be in the same room
def robot_move_CB(self, room_name, navigation_time):
    rospy.wait_for_service(service='robot_move_service')
    robot_move_proxy = rospy.ServiceProxy(name='robot_move_service', service_class=robot_move)

    try:
        if robot_move_proxy(room_name).reached:
            reached_time = rospy.get_rostime().to_time()

            rule_1_broken = False
            rule_2_broken = False
            rospy.wait_for_service(service='detect_frame')
            detection_service = rospy.ServiceProxy(name='detect_frame', service_class=yolo_detect)
            while rospy.get_rostime().to_time() - reached_time < navigation_time:
                cat_detected = False
                dog_detected = False
                for detection in detection_service().detections:
                    if room_name == 'D' and detection.name == 'Person':
                        rule_1_broken = True
                        rospy.loginfo('RULE 1 BROKEN !!!!!')
                    elif detection.name == 'Cat':
                        cat_detected = True
                    elif detection.name == 'Dog':
                        dog_detected = True

                if dog_detected and cat_detected:
                    rule_2_broken = True
                    rospy.loginfo('RULE 2 BROKEN !!!!!')

                pub = rospy.Publisher(name='/cmd_vel', data_class=Twist, queue_size=10, latch=True)
                velocity = Twist()
                velocity.linear.x = 2
                velocity.angular.z = math.radians(90)
                pub.publish(velocity)
            return 'succeeded'
        return 'aborted'
    except rospy.ServiceException as e:
        rospy.logerr('\nInvalid room name, either A, B or D')


def run():
    sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'], input_keys=['goal'], output_keys=['result'])

    with sm:
        it = Iterator(outcomes=['succeeded', 'preempted', 'aborted'],
                      it=lambda: range(0, sm.userdata.goal.times_to_check),
                      input_keys=[],
                      output_keys=[], exhausted_outcome='succeeded')

        with it:
            container_sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted', 'continue'])
            with container_sm:
                container_sm.add(label='room_check_A', state=CBState(cb=robot_move_CB, cb_args=['A', 30], outcomes=['succeeded', 'preempted', 'aborted']),
                                 transitions={'succeeded': 'room_check_B'})

                container_sm.add(label='room_check_B', state=CBState(cb=robot_move_CB, cb_args=['B', 30], outcomes=['succeeded', 'preempted', 'aborted']),
                                 transitions={'succeeded': 'room_check_D'})

                container_sm.add(label='room_check_D', state=CBState(cb=robot_move_CB, cb_args=['D', 10],
                                                                     outcomes=['succeeded', 'preempted', 'aborted']),
                                 transitions={'succeeded': 'continue'})


            Iterator.set_contained_state('CONTAINER_STATE', container_sm, loop_outcomes=['continue'])

        StateMachine.add('it', it, {'succeeded': 'succeeded', 'aborted': 'aborted'})

    asw = ActionServerWrapper(
        'server', RoomCheckAction,
        wrapped_container=sm,
        succeeded_outcomes=['succeeded'],
        aborted_outcomes=['aborted'],
        preempted_outcomes=['preempted'],
        goal_key='goal',
        result_key='result'
    )
    asw.run_server()
