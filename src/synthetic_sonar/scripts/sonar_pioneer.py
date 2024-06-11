#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32 # Assuming Sonar topic publishes Float32 messages
import os

# Define the path to the text file
file_path = os.path.join(os.path.expanduser('~'), 'sonar_data.txt')

def callback(data):
    rospy.loginfo("Sonar reading: %f", data.data)
    with open(file_path, 'a') as file:
        file.write(f"{data.data}\n")

def sonar_listener():
    rospy.init_node('sonar_listener', anonymous=True)
    rospy.Subscriber('/RosAria/sonar', Float32, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        sonar_listener()
    except rospy.ROSInterruptException:
        pass
