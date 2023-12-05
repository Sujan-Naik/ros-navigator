#!/usr/bin/env python3
import actionlib
import actionlib_msgs.msg
import move_base_msgs.msg
import rospy
import smach
import std_msgs
from actionlib_msgs.msg import GoalID, GoalStatus
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from smach import StateMachine, Iterator, Sequence, CBState
from smach_ros import SimpleActionState, ActionServerWrapper
from second_coursework.msg import RoomCheckAction, RoomCheckActionGoal
from second_coursework.srv import robot_move

def robot_move_CB(self,room_name):
    rospy.wait_for_service(service='robot_move_service')
    robot_move_proxy = rospy.ServiceProxy(name='robot_move_service', service_class=robot_move)
    try:
        if robot_move_proxy(room_name).reached == True:
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
                container_sm.add(label='room_check_A', state=CBState(cb=robot_move_CB, cb_args=['A'], outcomes=['succeeded', 'preempted', 'aborted']),
                                 transitions={'succeeded': 'room_check_B'})

                container_sm.add(label='room_check_B', state=CBState(cb=robot_move_CB, cb_args=['B'], outcomes=['succeeded', 'preempted', 'aborted']),
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
