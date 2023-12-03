#!/usr/bin/env python3
import actionlib
import actionlib_msgs.msg
import move_base_msgs.msg
import rospy
import smach
import std_msgs
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from smach import StateMachine, Iterator, Sequence
from smach_ros import SimpleActionState, ActionServerWrapper
from second_coursework.msg import RoomCheckAction, RoomCheckActionGoal

# class RoomCheckServer:
#   def __init__(self):
#     self.server = actionlib.SimpleActionServer(name='room_check', ActionSpec=RoomCheckAction, execute_cb=self.execute,auto_start=False)
#     self.server.start()
#
#   def execute(self, goal):
#     # Do lots of awesome groundbreaking robot stuff here
#     self.server.set_succeeded()
#
#
# if __name__ == '__main__':
#   rospy.init_node('room_check_server')
#   server = RoomCheckServer()


"""
Room A is at 1.5,8.5
Room B 6, 8.5
Room C 10.5, 8.5
Room D 1.5,3
Room E 6,3
Room F 10.5,3


"""
rospy.init_node('main_node')
try:
    rospy.init_node("time")
except rospy.exceptions.ROSException as e:
    print("Node has already been initialized")

class PoseGoalState(SimpleActionState):

    def __init__(self, x, y):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.stamp = rospy.get_rostime()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        self.goal.target_pose.pose.position.z = 0
        self.goal.target_pose.pose.orientation.x = 0
        self.goal.target_pose.pose.orientation.y = 0
        self.goal.target_pose.pose.orientation.z = 0
        self.goal.target_pose.pose.orientation.w = 1

        SimpleActionState.__init__(self, action_spec=RoomCheckAction, action_name='room_check', outcomes=['succeeded'],
                                   goal=self.goal)

    def execute(self, userdata):
        self.client.send_goal(self.goal)
        self.client.wait_for_result()
        return 'succeeded'


class TimerPoseGoalState(SimpleActionState):

    def __init__(self, x, y, stop_time):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.goal = MoveBaseGoal()

        self.goal.target_pose.header.stamp = rospy.get_rostime()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        self.goal.target_pose.pose.position.z = 0
        self.goal.target_pose.pose.orientation.x = 0
        self.goal.target_pose.pose.orientation.y = 0
        self.goal.target_pose.pose.orientation.z = 0
        self.goal.target_pose.pose.orientation.w = 1

        SimpleActionState.__init__(self, action_spec=RoomCheckAction, action_name='room_check', outcomes=['succeeded'],
                                   goal=self.goal, input_keys=['time_starting'], output_keys=['time_finishing'])

    def execute(self, userdata):
        self.client.send_goal(self.goal)
        self.client.wait_for_result()

        return 'succeeded'


class NavigationContainer(Sequence):

    def __init__(self, x, y):
        super().__init__(outcomes=['succeeded', 'aborted', 'preempted', 'continue'])
        self.add(label='north', state=PoseGoalState(x, y+1.5), transitions={'succeeded': 'east'})
        self.add(label='east', state=PoseGoalState(x + 1.5, y), transitions={'succeeded': 'south'})
        self.add(label='south', state=PoseGoalState(x, y - 1.5), transitions={'succeeded': 'west'})
        self.add(label='west', state=PoseGoalState(x - 1.5, y), transitions={'succeeded': 'north'})

def run():

    sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'], input_keys=['goal'], output_keys=['result'])

    with sm:

        it = Iterator(outcomes=['succeeded', 'preempted', 'aborted'], it=lambda: range(0, sm.userdata.goal.times_to_check),
                      input_keys=[],
                      output_keys=[], exhausted_outcome='succeeded')
        with it:
            container_sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted', 'continue'])
            with container_sm:
                container_sm.add(label='room_check_A', state=PoseGoalState(1.5, 8.5),
                                 transitions={'succeeded': 'room_check_B'})

                container_sm.add(label='room_check_B', state=PoseGoalState(6, 8.5),
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








