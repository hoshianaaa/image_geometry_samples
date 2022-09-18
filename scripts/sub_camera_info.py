import rospy
from sensor_msgs.msg import CameraInfo, Image


def callback(msg):
  print(msg) 

rospy.init_node("camera_info_sub")
image_sub = rospy.Subscriber("camera/color/camera_info", CameraInfo, callback)
rospy.spin()
