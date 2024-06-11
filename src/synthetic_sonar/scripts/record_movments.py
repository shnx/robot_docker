#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import signal
import sys

class MovementRecorder:
    def __init__(self):
        rospy.init_node('movement_recorder', anonymous=True)
        self.movement_publisher = rospy.Publisher('/pioneer/cmd_vel', Twist, queue_size=10)
        self.recording = False
        self.movements = []
        self.is_recording_completed = False

    def record_movement(self, twist):
        if self.recording:
            self.movements.append(twist)

    def start_recording(self):
        self.recording = True
        rospy.loginfo("Recording started. Press Ctrl+C to stop.")
        while not rospy.is_shutdown() and not self.is_recording_completed:
            rospy.sleep(0.1)  # Adjust sleep duration based on your robot's response time
        self.recording = False
        rospy.loginfo("Recording stopped.")

    def save_movements(self, filename='recorded_movements.txt'):
        with open(filename, 'w') as file:
            for twist in self.movements:
                file.write(f'{twist.linear.x} {twist.angular.z}\n')

def signal_handler(sig, frame):
    recorder.is_recording_completed = True

if __name__ == '__main__':
    recorder = MovementRecorder()

    # Subscribe to the /cmd_vel topic
    rospy.Subscriber('/pioneer/cmd_vel', Twist, recorder.record_movement)

    # Set up a signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    # Trigger recording until Ctrl+C is pressed
    recorder.start_recording()

    # Save recorded movements to a file
    recorder.save_movements()
