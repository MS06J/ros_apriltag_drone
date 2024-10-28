import rospy
from apriltag_pose.msg import Poses

def print_msg(msg: Poses):
    rospy.loginfo(msg)

class PoseListener:
    def __init__(self, topic='poses', action=print_msg):
        rospy.init_node('pose_listner')
        self.sub = rospy.Subscriber('poses', Poses, callback=action)
        rospy.loginfo('Node has been started')

if __name__ == '__main__':
    listner = PoseListener()
    rospy.spin()