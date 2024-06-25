#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64, Bool, String
from sensor_msgs.msg import Image
import random
import numpy as np
import cv2
from cv_bridge import CvBridge

class RandomDataPublisher:
    def __init__(self):
        rospy.init_node('random_data_publisher', anonymous=True)

        # Publishers for temperature and pressure sensors
        self.temperature_pubs = [rospy.Publisher(f'/temperature_{i}', Float64, queue_size=10) for i in range(5)]
        self.pressure_pubs = [rospy.Publisher(f'/pressure_{i}', Float64, queue_size=10) for i in range(5)]

        # Publishers for gas detection, motor status, alarm status, and camera image
        self.gas_detected_pub = rospy.Publisher('/gas_detected', Bool, queue_size=10)
        self.motor_status_pub = rospy.Publisher('/motor_status', String, queue_size=10)
        self.alarm_status_pub = rospy.Publisher('/alarm_status', String, queue_size=10)
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)

        self.bridge = CvBridge()
        self.rate = rospy.Rate(1)  # 1 Hz

        # Initialize state variables
        rospy.Subscriber('/motor_command', String, self.motor_command_callback)
        rospy.Subscriber('/gas_detected', Bool, self.gas_detected_callback)
        self.motor_on = False
        self.alarm_active = False

    def motor_command_callback(self, msg):
        command = msg.data
        rospy.loginfo(f"Received motor command: {command}")  # Log the received command
        if command == 'turn_on':
            self.motor_on = True
            self.motor_status_pub.publish('running')
            rospy.loginfo("Motor turned on")
        elif command == 'turn_off':
            self.motor_on = False
            self.motor_status_pub.publish('stopped')
            rospy.loginfo("Motor turned off")

    def gas_detected_callback(self, msg):
        gas_detected = msg.data
        rospy.loginfo(f"Gas detected: {gas_detected}")  # Log the gas detection state
        if gas_detected:
            self.motor_on = False
            self.motor_status_pub.publish('stopped')
            self.alarm_active = True
            self.alarm_status_pub.publish('active')
            rospy.loginfo("Gas detected: Motor turned off and alarm activated")
        elif not gas_detected and self.alarm_active:
            self.alarm_active = False
            self.alarm_status_pub.publish('inactive')
            rospy.loginfo("Gas dissipated: Alarm deactivated")

    def publish_random_data(self):
        while not rospy.is_shutdown():
            # Generate random data for sensors
            temperatures = [random.uniform(10.0, 100.0) for _ in range(5)]
            pressures = [random.uniform(1.0, 10.0) for _ in range(5)]

            # Simulate gas detection based on some condition
            # Replace this with your actual gas detection logic
            gas_detected = random.choice([True, False])

            motor_status = 'running' if self.motor_on else 'stopped'
            alarm_status = 'active' if self.alarm_active else 'inactive'

            # Create a random image
            random_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            image_message = self.bridge.cv2_to_imgmsg(random_image, encoding="bgr8")

            # Log the data being published
            rospy.loginfo(f"Publishing: Temperatures={temperatures}, Pressures={pressures}, Gas Detected={gas_detected}, Motor Status={motor_status}, Alarm Status={alarm_status}")

            # Publish the temperature and pressure data
            for i in range(5):
                self.temperature_pubs[i].publish(temperatures[i])
                self.pressure_pubs[i].publish(pressures[i])

            # Publish the gas detection, motor status, alarm status, and camera image data
            self.gas_detected_pub.publish(gas_detected)
            self.motor_status_pub.publish(motor_status)
            self.alarm_status_pub.publish(alarm_status)
            self.image_pub.publish(image_message)

            # Sleep to maintain the loop rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = RandomDataPublisher()
        publisher.publish_random_data()
    except rospy.ROSInterruptException:
        pass
