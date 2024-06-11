#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
import time
import numpy as np
import matplotlib.pyplot as plt

class SonarSubscriber:
    def __init__(self):
        self.sonar1_data = []
        self.sonar2_data = []

        rospy.init_node('sonar_copmare', anonymous=True)

        rospy.Subscriber('/Sonar_back_90', Range, self.sonar_callback, callback_args=self.sonar1_data)
        rospy.Subscriber('/Sonar_back_90_left', Range, self.sonar_callback, callback_args=self.sonar2_data)

        self.start_time = time.time()
        self.duration = 1  # Listen for 2 seconds

        rospy.spin()

        # Save data to files
        np.savetxt('sonar1_data.txt', np.array(self.sonar1_data))
        np.savetxt('sonar2_data.txt', np.array(self.sonar2_data))

        # Plot the data
        self.plot_data()

    def sonar_callback(self, data, sonar_data):
        sonar_data.append(data.range)

        # Check if the duration has passed
        if time.time() - self.start_time >= self.duration:
            rospy.signal_shutdown('Duration reached')

    def plot_data(self):
        sonar1_data = np.loadtxt('sonar1_data.txt')
        sonar2_data = np.loadtxt('sonar2_data.txt')

        sonar1_average = np.mean(sonar1_data)
        sonar2_average = np.mean(sonar2_data)

        #plt.plot(sonar1_data, label='Sonar values from our proposed sensor model', color='blue')
        #plt.plot(sonar2_data, label='Sonar values form Gazebo sonar plugin', color='orange')
        plt.scatter(range(len(sonar1_data)), sonar1_data, label='Synthetic Sonar Model', color='blue')
        plt.scatter(range(len(sonar2_data)), sonar2_data, label='Gazebo Sonar plugin', color='orange')
    
        plt.axhline(y=sonar1_average, color='blue', linestyle='--', label=' Average')
        plt.axhline(y=sonar2_average, color='orange', linestyle='--', label='Average')

        plt.xlabel('Time (samples)')
        plt.ylabel('Sonar Range')
        plt.legend()
        plt.title('Sonar Ranges Comparison')
        plt.show()

if __name__ == '__main__':
    try:
        sonar_subscriber = SonarSubscriber()
    except rospy.ROSInterruptException:
        pass
