#!/usr/bin/env python3
import rospy
import cv2
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    rospy.init_node('video_publisher', anonymous=True)
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    bridge = CvBridge()

    video_path = '/home/saadman/Videos/simplescreenrecorder-২০২৪-০৬-২৫_০০.২৭.৪৪.mkv'
 # Change this to your MKV file path
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        rospy.logerr("Error: Unable to open video file")
        return

    rate = rospy.Rate(30)  # Adjust the rate as per your video's frame rate

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.loginfo("End of video file")
            break

        msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_pub.publish(msg)

        time.sleep(0.033)  # Add a sleep to control the frame rate, here it's set to approximately 30 fps

        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
