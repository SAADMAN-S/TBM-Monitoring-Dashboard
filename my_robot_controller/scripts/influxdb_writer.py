#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64, Bool, String
from sensor_msgs.msg import Image
from influxdb import InfluxDBClient
from cv_bridge import CvBridge
import numpy as np
import cv2

# InfluxDB settings
client = InfluxDBClient(host='localhost', port=8086, username='Saadman', password='saadman1234', database='my_robot_data')

bridge = CvBridge()

def temperature_callback(msg, args):
    index = args
    json_body = [
        {
            "measurement": "temperature",
            "tags": {
                "sensor": f"temperature_{index}"
            },
            "fields": {
                "value": msg.data
            }
        }
    ]
    client.write_points(json_body)

def pressure_callback(msg, args):
    index = args
    json_body = [
        {
            "measurement": "pressure",
            "tags": {
                "sensor": f"pressure_{index}"
            },
            "fields": {
                "value": msg.data
            }
        }
    ]
    client.write_points(json_body)

def gas_detected_callback(msg):
    json_body = [
        {
            "measurement": "gas_detected",
            "fields": {
                "value": msg.data
            }
        }
    ]
    client.write_points(json_body)

def motor_status_callback(msg):
    json_body = [
        {
            "measurement": "motor_status",
            "fields": {
                "value": msg.data
            }
        }
    ]
    client.write_points(json_body)

def alarm_status_callback(msg):
    json_body = [
        {
            "measurement": "alarm_status",
            "fields": {
                "value": msg.data
            }
        }
    ]
    client.write_points(json_body)

def image_callback(msg):
    # Convert ROS Image message to OpenCV image
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    # For simplicity, let's store the mean color value as an example
    mean_color = cv2.mean(cv_image)[:3]
    json_body = [
        {
            "measurement": "camera_image",
            "fields": {
                "mean_red": mean_color[2],
                "mean_green": mean_color[1],
                "mean_blue": mean_color[0]
            }
        }
    ]
    client.write_points(json_body)

def influxdb_writer():
    rospy.init_node('influxdb_writer', anonymous=True)

    for i in range(5):
        rospy.Subscriber(f'/temperature_{i}', Float64, temperature_callback, i)
        rospy.Subscriber(f'/pressure_{i}', Float64, pressure_callback, i)

    rospy.Subscriber('/gas_detected', Bool, gas_detected_callback)
    rospy.Subscriber('/motor_status', String, motor_status_callback)
    rospy.Subscriber('/alarm_status', String, alarm_status_callback)
    rospy.Subscriber('/camera/image_raw', Image, image_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        influxdb_writer()
    except rospy.ROSInterruptException:
        pass
