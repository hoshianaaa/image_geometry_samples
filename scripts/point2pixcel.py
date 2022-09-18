#!/usr/bin/env python
#coding: utf-8

import sys
import rospy, tf2_ros, message_filters
import image_geometry

from sensor_msgs.msg import CameraInfo, Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

class FrameDrawer(object):
    def __init__(self, frame_ids, input_image_topic, input_camera_info_topic, output_topic):
        self.frame_ids = frame_ids
        image_topic = rospy.resolve_name(input_image_topic)
        info_topic = rospy.resolve_name(input_camera_info_topic)
        image_sub = message_filters.Subscriber(image_topic, Image)
        info_sub = message_filters.Subscriber(info_topic, CameraInfo)
        self.ts = message_filters.TimeSynchronizer([image_sub, info_sub], 10)
        self.ts.registerCallback(self.imageCb)

#        self.pub_ = rospy.Publisher(output_topic, Image, queue_size=1)

        self.cam_model_ = image_geometry.PinholeCameraModel()

        self.FONT = cv2.FONT_HERSHEY_SIMPLEX
        self.FONT_SCALE = 1
        self.FONT_THICKNESS = 3
        self.COLOR = (0,0,255) #BGR

    def imageCb(self, image_msg, info_msg):
        cv_bridge = CvBridge()
        try:
            image = cv_bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr("[draw_frames] Failed to convert image")

        self.cam_model_.fromCameraInfo(info_msg)

        frame_id = info_msg.header.frame_id

        pt = [0.44, -0.059, -0.073]
        uv = self.cam_model_.project3dToPixel(pt)
        uv = (int(uv[0]), int(uv[1]))

        print(uv)

        RADIUS = 6
        #cv2.circle(image, uv, RADIUS, self.COLOR, -1)
        (text_size, baseline) = cv2.getTextSize(frame_id, self.FONT, self.FONT_SCALE, self.FONT_THICKNESS)
        origin = (uv[0] - text_size[0]/2,
                  uv[1] - RADIUS - baseline - 3)
        #cv2.putText(image, frame_id, origin, self.FONT, self.FONT_SCALE, self.COLOR, self.FONT_THICKNESS)

        #self.pub_.publish( cv_bridge.cv2_to_imgmsg(image, "bgr8") )

if __name__ == '__main__':
    rospy.init_node("draw_frames")

    frame_ids = sys.argv[1:]
    fd = FrameDrawer(frame_ids, "/camera/color/image_raw", "/camera/color/camera_info", "out_image")
    while not rospy.is_shutdown():
        pass
