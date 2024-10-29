import rospy
import scipy as sp
from mavlink_sender import MavlinkSender
from pose_listener import PoseListener



sender = MavlinkSender('udp:192.168.1.166:14551')

sender.set_ekf_home()
sender.set_home()

listener = PoseListener(action=sender.send_pose)
rospy.spin()
