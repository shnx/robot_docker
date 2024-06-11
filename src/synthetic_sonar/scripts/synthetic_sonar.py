#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan, Range
import random

class SyntheticSonarNode:
    def __init__(self):
        self.angle_range = 20
        self.angle_increment = None
        self.measurement_angle_array = None
        self.probabilities_unnormalized = None
        self.debug=True
        self.std_deviation = 10
        self.mean_angle = 0 
        self.setup()

    def setup(self):
         
        self.gaussian_angles = np.linspace(-10, 10, 1000)
        
        self.probabilities_unnormalized = np.exp(-(self.gaussian_angles - self.mean_angle)**2 / (2 * self.std_deviation**2))*0.9
        rospy.init_node('synthetic_sonar_node', anonymous=True)
        self.setup_publishers()
        rospy.Subscriber("/base_scan", LaserScan, self.callback)

        # Initialize other variables and data here

    def setup_publishers(self):
        self.sonar_publishers = {}
        self.sonar_laser_publishers = {}
        
            
        

        sensor_frame_data = [
            
            ('Sonar_front_90_right_link', 0, 180,0.1),
            ('Sonar_front_50_right_link', 190, 270,0.02),
        ]

       
        for frame_id, start_idx, end_idx,diff in sensor_frame_data:
            
            topic_name_sonar = '/Sonar_' + frame_id.split('_')[1]+'_' +frame_id.split('_')[2]
            topic_name_laser = '/Sonar_Laser_' + frame_id.split('_')[1]+'_' +frame_id.split('_')[2]

            self.sonar_publishers[frame_id] = {
                'sonar_publisher': rospy.Publisher(topic_name_sonar, Range, queue_size=10),
                'start_idx': start_idx,
                'end_idx': end_idx,
                'laser_publisher': rospy.Publisher(topic_name_laser, LaserScan, queue_size=10),
                'diff':diff
            }
          
        
        """  if(self.debug==True):# for debugging

            for frame_id, start_idx, end_idx in sensor_frame_data: """
                
                
            
            

    def rejection_sampling(self, measurements_angles, start_idx, end_idx):
        angle_start = start_idx * self.angle_increment
        angle_end = end_idx * self.angle_increment
        sorted_by_measurement = sorted(measurements_angles, key=lambda x: x[0])
        min_measurement = min(measurements_angles, key=lambda x: x[0])
      
        for i in range(end_idx-start_idx):
            measurement, angle = sorted_by_measurement[i]
            
            random_uniform = random.uniform(0, 1)# get another random number each time
            #print(random_uniform)
            if random_uniform <= 0.5:
                return measurement
        
    






        
        

    def callback(self, data):
        #if self.angle_increment is None:
        self.angle_increment = data.angle_increment * 180.0 / 3.14159
        #if self.measurement_angle_array is None:
        temp = np.copy(data.ranges)
        self.measurement_angle_array = list(zip(data.ranges, np.arange(0, len(data.ranges)) * self.angle_increment))

        # Implement the callback logic here, including range_values

        for frame_id, data_dict in self.sonar_publishers.items():
            fake_sonar = Range()
            fake_sonar.header.stamp = rospy.Time.now()
            fake_sonar.field_of_view = self.angle_range * 3.14159 / 180.0
            fake_sonar.min_range = 0.1
            fake_sonar.max_range = 30
            fake_sonar.header.frame_id = frame_id

            start_idx = data_dict['start_idx']
            end_idx = data_dict['end_idx']
            #print("*******************************************")
            #print(len(data.ranges))
            #print(start_idx)#
            #print(end_idx)
            #print(temp)
            #print("*******************************************")
            range_value = self.rejection_sampling(self.measurement_angle_array[start_idx:end_idx], start_idx, end_idx)+data_dict['diff']
            #print(range_value)
            

            fake_sonar.range = range_value
            if self.debug:
            #for frame_id, data_dict in self.sonar_laser_publishers.items():
            # Create and publish a laser scan when debug is true
                part_scan = LaserScan()
                part_scan.header.frame_id = "laser"
                #back_up=data.ranges
                part_scan=data
                part_scan.header.stamp = rospy.Time.now()

                #start_idx = data_dict['start_idx']
                #end_idx = data_dict['end_idx']
                
                
                temp2 = np.copy(temp) * 0
                temp2[start_idx:end_idx] = temp[start_idx:end_idx]
                part_scan.ranges = temp2
                #print("*******************************************")
                #print(frame_id)
                #print(back_up)
                #print(start_idx)
                #print(end_idx)
                #print (temp2[start_idx:end_idx])
                #print("*******************************************")
                data_dict['laser_publisher'].publish(part_scan)  

            data_dict['sonar_publisher'].publish(fake_sonar)

        

    





if __name__ == '__main__':
    try:
        node = SyntheticSonarNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
