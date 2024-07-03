#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from typing import Optional
from sensor_msgs.msg import CameraInfo, Image
import numpy as np
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

class capture_scene:


   def __init__(self):
       return


   def callback(data):
       cap = capture_scene()
       cap.capture_image_and_save_info()


   def rgb_callback(data):
       global rgb_array
       rgb_array = bridge.imgmsg_to_cv2(data, "bgr8")
       rospy.sleep(1)


   def depth_callback(data):
       global depth_array
       depth_img = bridge.imgmsg_to_cv2(data, "32FC1")
       depth_array = np.array(depth_img, dtype=np.dtype("f8"))
       rospy.sleep(1)


   def camera_info_callback(data):
       global camera_info
       cam_info = data.K
       camera_info = np.array(
           [
               [cam_info[0], 0.0, cam_info[2]],
               [0.0, cam_info[4], cam_info[5]],
           ]
       )
       rospy.sleep(1)


   def save_png(self):
       rospy.Subscriber("/camera/color/image_raw", Image, capture_scene.png_callback)
       rospy.sleep(2)


   def filter_color(self, color_img: np.ndarray, depth_data: np.ndarray) -> np.ndarray:
        lower_blue = np.array([110, 50, 50])
        upper_blue = np.array([130, 255, 255])
        lower_brown = np.array([10, 20, 20])
        upper_brown = np.array([52, 255, 255])
        hsv_image = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
        mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)
        mask_brown = cv2.inRange(hsv_image, lower_brown, upper_brown)
        mask_combined = cv2.bitwise_or(mask_blue, mask_brown)
        mask_non_brown_blue = cv2.bitwise_not(mask_brown)
        non_red_blue_objects_hsv = cv2.bitwise_and(
            hsv_image, hsv_image, mask=mask_non_brown_blue
        )
        filtered_depth_data = cv2.bitwise_and(
            depth_data, depth_data, mask=mask_non_brown_blue
        )
        return filtered_depth_data


   def capture_image_and_save_info(self) -> str:
       rospy.Subscriber("/camera/color/image_raw", Image, capture_scene.rgb_callback)
       rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, capture_scene.depth_callback)
       rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, capture_scene.camera_info_callback)
       rospy.sleep(5)
       filtered_depth_array = capture_scene.filter_color(self, np.array(rgb_array), np.array(depth_array))
       data_dict = {
           "rgb": np.array(rgb_array),
           "depth_raw": depth_array / 1000.0,
           "depth": filtered_depth_array / 1000.0,
           "label": np.zeros((720, 1280), dtype=np.uint8),
           "K": camera_info,
       }
       save_dir = '/home/davinasanghera/contact_graspnet/contact_graspnet/test_data/scene.npy' # CHANGE TO YOUR OWN PATH
       rospy.loginfo(save_dir)
       np.save(save_dir, data_dict)
       return save_dir


if __name__ == "__main__":
   rospy.init_node("capture_scene_node")

   rospy.Subscriber("capture_reached", Bool, capture_scene.callback)
   rospy.spin()
