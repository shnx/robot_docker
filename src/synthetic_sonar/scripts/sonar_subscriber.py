import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
import numpy as np


from tf import TransformListener
from tf.transformations import euler_from_quaternion

class SonarSubscriber:
    def __init__(self,opening_angle=16):

        self.opening_angle=opening_angle
        self.min_range=0.01
        self.max_range=30.0
        self.sensor_frame_data = {
            'Sonar_front_90_left_link': [None, None, None, None],
            'Sonar_front_50_left_link': [None, None, None, None],#
            'Sonar_front_90_right_link': [None, None, None, None],
            'Sonar_front_50_right_link': [None, None, None, None],
           
        }
        """ 'Sonar_front_30_right_link': [None, None, None, None],
            'Sonar_front_10_right_link': [None, None, None, None],
            'Sonar_front_10_left_link': [None, None, None, None],#Sonar_back_10_left_link
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
            'Sonar_back_90_left_link': [None, None, None, None],"""
        self.sonar_topics = [
            'Sonar_front_90_left', 'Sonar_front_50_left','Sonar_front_90_right', 'Sonar_front_50_right'
        ]
        """, 'Sonar_front_30_right', 'Sonar_front_10_right',
            'Sonar_front_10_left', 'Sonar_front_30_left', 'Sonar_front_50_left', 'Sonar_front_90_left',
            'Sonar_back_90_right', 'Sonar_back_50_right', 'Sonar_back_30_right', 'Sonar_back_10_right',
            'Sonar_back_10_left', 'Sonar_back_30_left', 'Sonar_back_50_left', 'Sonar_back_90_left'"""
        self.tf_listener = TransformListener()
        self.sonar_data = {}
        self.sonar_positions = {}
        self.sonar_angles = {}
        #rospy.init_node('sonar_subscriber', anonymous=True)
        self.setup_sonar_arrays()

        

        for topic in self.sonar_topics:
            rospy.Subscriber(topic, Range, self.sonar_callback, callback_args=topic)

    def sonar_callback(self, data, topic):
        if (data.range is not None):
            data.range=np.clip(data.range,self.min_range,self.max_range)
        self.sonar_data[topic] = data.range
    
    def extract_angle_from_topic(self, topic):
        frame_id=topic+'_link'
        angle=self.sensor_frame_data[frame_id][2]
        return angle
        # Split the topic name and extract the angle part
        #angle_str = topic.split('_')[-2]  # Assuming angle is the second-to-last part in the topic name
        
        #angle = int(angle_str)
            

    def generate_measurement(self):
        measurement_data = {}
        for topic in self.sonar_topics:
            if topic in self.sonar_data:
                measurement_data[topic] = self.sonar_data[topic]
            else:
                measurement_data[topic] = None

        return measurement_data
    def setup_sonar_arrays(self):
        for frame_id in self.sensor_frame_data:
            try:
                #frame_id="Sonar_back_90_left_link"
                rospy.sleep(0.30) 
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

