#!/usr/bin/env python3
import math

import actionlib
import actionlib_msgs.msg
import geometry_msgs.msg
import move_base_msgs.msg
import rospy
import smach
import std_msgs
from actionlib_msgs.msg import GoalID, GoalStatus
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from rospy import Duration
from smach import StateMachine, Iterator, Sequence, CBState, Concurrence
from smach_ros import SimpleActionState, ActionServerWrapper
from std_msgs.msg import String

from second_coursework.msg import RoomCheckAction, RoomCheckActionGoal, RoomCheckActionResult, RoomCheckActionFeedback, \
    RoomCheckFeedback, RoomCheckResult
from second_coursework.srv import robot_move, yolo_detect


# Rule 1: No people should be in the kitchen (Room D) at any time
# Rule 2: Cat and dog shouldn't be in the same room
def robot_move_CB(self, room_name, navigation_time):
    rospy.wait_for_service(service='robot_move_service')
    robot_move_proxy = rospy.ServiceProxy(name='robot_move_service', service_class=robot_move, persistent=True)

    try:
        if robot_move_proxy(room_name).reached:
            reached_time = rospy.get_rostime().to_time()

            rule_1_broken = False
            rospy.wait_for_service(service='/detect_frame')
            detection_service = rospy.ServiceProxy(name='/detect_frame', service_class=yolo_detect)
            while rospy.get_rostime().to_time() - reached_time < navigation_time:
                cat_detected = False
                dog_detected = False
                for detection in detection_service().detections:
                    if (not rule_1_broken) and room_name == 'D' and detection.name == 'person':
                        rule_1_broken = True
                        self.result.rule_break_amount[0] = self.result.rule_break_amount[0] + 1
                        try:
                            message = rospy.wait_for_message(topic='move_base/feedback', topic_type=PoseStamped, timeout=rospy.rostime.Duration(secs=5))
                            publish_feedback(self, 1, message)
                        except rospy.ROSInterruptException:
                            rospy.logerr_once('No feedback on move_base/feedback to obtain position, rule break 1 discarded')

                    if detection.name == 'cat':
                        cat_detected = True
                    if detection.name == 'dog':
                        dog_detected = True

                if dog_detected and cat_detected:
                    self.result.rule_break_amount[1] = self.result.rule_break_amount[1] + 1
                    try:
                        message = rospy.wait_for_message(topic='move_base/feedback', topic_type=PoseStamped,
                                                         timeout=rospy.rostime.Duration(secs=5))
                        publish_feedback(self, 2, message)
                    except rospy.ROSInterruptException:
                        rospy.logerr_once('No feedback on move_base/feedback to obtain position, rule break 2 discarded')

                robot_move_proxy(room_name)

            return 'succeeded'
        return 'aborted'
    except rospy.ServiceException as e:
        rospy.logerr('\nInvalid room name, either A, B or D')


sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'], input_keys=['goal'],
                  output_keys=['feedback', 'result'])

with sm:
    it = Iterator(outcomes=['succeeded', 'preempted', 'aborted'],
                  it=lambda: range(0, sm.userdata.goal.times_to_check + 1),
                  input_keys=[],
                  output_keys=['result'], exhausted_outcome='succeeded')

    with it:
        container_sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted', 'continue'],
                                    output_keys=['feedback', 'result'])
        container_sm.userdata.feedback = RoomCheckFeedback()
        container_sm.userdata.result = RoomCheckResult()
        container_sm.userdata.result.rule_break_amount = [0, 0]
        with container_sm:
            container_sm.add(label='room_check_A',
                             state=CBState(cb=robot_move_CB,
                                           cb_args=['A', 30],
                                           outcomes=['succeeded', 'preempted', 'aborted'],
                                           io_keys=['feedback', 'result']),
                             transitions={'succeeded': 'room_check_B'}),

            container_sm.add(label='room_check_B',
                             state=CBState(cb=robot_move_CB,
                                           cb_args=['B', 30],
                                           outcomes=['succeeded', 'preempted', 'aborted'],
                                           io_keys=['feedback', 'result']),
                             transitions={'succeeded': 'room_check_D'})

            container_sm.add(label='room_check_D',
                             state=CBState(cb=robot_move_CB,
                                           cb_args=['D', 10],
                                           outcomes=['succeeded', 'preempted', 'aborted'],
                                           io_keys=['feedback', 'result']),
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
    feedback_key='feedback',
    result_key='result'
)


def publish_feedback(self, rule_broke, message):
    self.feedback.robot_position = message.feedback.base_position.pose.position
    self.feedback.rule_broken = rule_broke
    ActionServerWrapper.publish_feedback(self=asw, userdata=self)
    pub = rospy.Publisher('/tts/phase', String, queue_size=1)
    if rule_broke == 1:
        pub.publish('Person leave')
    else:
        pub.publish('Cats and dogs leave')


def run():
    asw.run_server()
