#!/usr/bin/env python3
import rospy
from tf import TransformListener
from tf.transformations import euler_from_quaternion
import math

class SonarArray:
    def __init__(self):
        #rospy.init_node('sonar_arrays_node')

        self.tf_listener = TransformListener()

        # Sensor frame data with optional start/end index and diff
        self.sensor_frame_data = {
            'Sonar_front_90_right_link': [None, None, None, None],
            'Sonar_front_50_right_link': [None, None, None, None],
            'Sonar_front_30_right_link': [None, None, None, None],
            'Sonar_front_10_right_link': [None, None, None, None],
            'Sonar_front_10_left_link': [None, None, None, None],
            'Sonar_front_30_left_link': [None, None, None, None],
            'Sonar_front_50_left_link': [None, None, None, None],
            'Sonar_front_90_left_link': [None, None, None, None],
            'Sonar_back_90_right_link': [None, None, None, None],
            'Sonar_back_50_right_link': [None, None, None, None],
            'Sonar_back_30_right_link': [None, None, None, None],
            'Sonar_back_10_right_link': [None, None, None, None],
            'Sonar_back_10_left_link': [None, None, None, None],
            'Sonar_back_30_left_link': [None, None, None, None],
            'Sonar_back_50_left_link': [None, None, None, None],
            'Sonar_back_90_left_link': [None, None, None, None],
        }

        # Initialize positions and angles
        self.sonar_positions = {}
        self.sonar_angles = {}
        self.setup_sonar_arrays()
        #print(self.sensor_frame_data)

        

    def setup_sonar_arrays(self):
        for frame_id in self.sensor_frame_data:
            try:
                #frame_id="Sonar_back_90_left_link"
                rospy.sleep(0.10) 
                # Get the transform from the laser frame to the sonar frame
                (trans, rot) = self.tf_listener.lookupTransform("laser", frame_id, rospy.Time(0))
                #self.tf_listener = TransformListener(rospy.Duration(10))


                # Extract the position (translation) from the transform
                position = {'x': trans[0], 'y': trans[1], 'z': trans[2]}

                # Extract the yaw angle from the quaternion orientation
                _, _, yaw = euler_from_quaternion(rot)

                # Save the position and angle
                self.sonar_positions[frame_id] = position
                self.sonar_angles[frame_id] = yaw
                self.sensor_frame_data[frame_id][1] = position
                self.sensor_frame_data[frame_id][2] = yaw
            except Exception as e:
                print(f"Error setting up {frame_id}: {str(e)}")

        
    def get_position(self):
        
        return self.sonar_positions
    def get_angle(self):
        
        return self.sonar_angles






if __name__ == '__main__':
    sonar_arrays = SonarArray()

    rate = rospy.Rate(1)  # Adjust the rate as needed

    while not rospy.is_shutdown():
        rospy.loginfo("Sonar Positions: {}".format(sonar_arrays.sonar_positions))
        rospy.loginfo("Sonar Angles: {}".format(sonar_arrays.sonar_angles))

        rate.sleep()
