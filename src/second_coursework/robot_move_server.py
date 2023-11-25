import rospy
from second_coursework.srv import robot_move
from move_base_msgs.msg import MoveBaseActionGoal


def move(request):
    pub = rospy.Publisher(name='/move_base/goal',data_class=MoveBaseActionGoal,queue_size=10)
    pub.publish(request)

def initialiseMovement():
    rospy.init_node(name='robot_move_server', anonymous=True)
    rospy.Service(name='robot_move', service_class=robot_move, handler=move)
    rospy.spin()


if __name__ == '__main__':
    try:
        initialiseMovement()
    except rospy.ROSInterruptException:
        pass
