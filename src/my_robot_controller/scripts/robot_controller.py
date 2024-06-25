#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Bool, Float64

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller')

        self.motor_status_pub = rospy.Publisher('/motor_status', String, queue_size=10)
        self.alarm_status_pub = rospy.Publisher('/alarm_status', String, queue_size=10)
        self.alert_pub = rospy.Publisher('/visual_alert', String, queue_size=10)  # Publisher for visual alerts

        rospy.Subscriber('/motor_command', String, self.motor_command_callback)
        rospy.Subscriber('/alarm_command', String, self.alarm_command_callback)
        rospy.Subscriber('/temperature_0', Float64, self.temperature_callback)  # Add other temperature sensors as needed
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

    def alarm_command_callback(self, msg):
        command = msg.data
        rospy.loginfo(f"Received alarm command: {command}")  # Log the received command
        if command == 'activate':
            self.alarm_active = True
            self.alarm_status_pub.publish('active')
            rospy.loginfo("Alarm activated")
        elif command == 'deactivate':
            self.alarm_active = False
            self.alarm_status_pub.publish('inactive')
            rospy.loginfo("Alarm deactivated")

    def temperature_callback(self, msg):
        temperature = msg.data
        if temperature > 80:
            alert_message = f"High temperature detected: {temperature}°C. Please turn off the motor."
            self.alert_pub.publish(alert_message)
            rospy.loginfo(alert_message)
            # Here, we don't stop the motor as per your requirement

    def gas_detected_callback(self, msg):
        gas_detected = msg.data
        rospy.loginfo(f"Gas detected: {gas_detected}")  # Log the gas detection state
        if gas_detected :
            self.motor_on = False
            self.motor_status_pub.publish('stopped')
            self.alarm_active = True
            self.alarm_status_pub.publish('active')
            rospy.loginfo("Gas detected: Motor turned off and alarm activated")
        elif not gas_detected and self.alarm_active:
            self.alarm_active = False
            self.alarm_status_pub.publish('inactive')
            rospy.loginfo("Gas dissipated: Alarm deactivated")

if __name__ == '__main__':
    try:
        controller = RobotController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
