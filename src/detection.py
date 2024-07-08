#!/usr/bin/env python3

import rospy
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os
import rospkg

def get_aruco_dict(marker_type):
    aruco_dicts = {
        '4x4': aruco.DICT_4X4_50,
        '5x5': aruco.DICT_5X5_100,
        '6x6': aruco.DICT_6X6_250,
        '7x7': aruco.DICT_7X7_1000,
        'original': aruco.DICT_ARUCO_ORIGINAL,
    }
    return aruco.Dictionary_get(aruco_dicts.get(marker_type, aruco.DICT_6X6_250))

def detect_aruco_markers(frame, aruco_dict):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    return corners, ids

def main():
    rospy.init_node('aruco_detector', anonymous=True)
    image_pub = rospy.Publisher('/aruco_detection/image', Image, queue_size=10)
    bridge = CvBridge()

    # Get the marker type parameter from ROS parameter server
    marker_type = rospy.get_param('~marker_type', '6x6')

    # Get the video path parameter and resolve the full path
    video_path_param = rospy.get_param('~video_path', 'videos/video1.mp4')
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('aruco_detection')
    video_path = os.path.join(package_path, video_path_param)

    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        rospy.logerr("Error opening video file: {}".format(video_path))
        return

    aruco_dict = get_aruco_dict(marker_type)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown() and cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        corners, ids = detect_aruco_markers(frame, aruco_dict)
        if ids is not None:
            frame = aruco.drawDetectedMarkers(frame, corners, ids)

        image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        image_pub.publish(image_msg)

        # Display the frame
        cv2.imshow('Aruco Marker Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
