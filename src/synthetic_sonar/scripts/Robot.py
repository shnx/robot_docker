import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np

class Robot:
    def __init__(self, topic_name='/gazebo/gt_pose'):
        
        self.topic_name = topic_name
        self.position = None
        self.yaw_angle = None

        #rospy.init_node('robot_listener', anonymous=True)
        rospy.Subscriber(self.topic_name, Odometry, self.odometry_callback)

    def odometry_callback(self, msg):
        # Extract position
        self.position = msg.pose.pose.position

        # Extract quaternion orientation
        orientation = msg.pose.pose.orientation

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        # Extract yaw angle
        self.yaw_angle = yaw

    def get_position(self):
        return self.position

    def get_yaw_angle(self):
        return self.yaw_angle
    #def update(self):
        # Manually trigger a callback to update the position
        #rospy.spin()

    def __call__(self):
        # Call the instance to get the latest data
        self.update()
        return self.get_position(), self.get_yaw_angle()

# if __name__ == "__main__":
#     robot = Robot()

#     while not rospy.is_shutdown():
#         # Get and print the position and yaw angle
#         position = robot.get_position()
#         yaw_angle = robot.get_yaw_angle()

#         if position is not None and yaw_angle is not None:
#             print(f"Position: ({position.x}, {position.y}, {position.z}), Yaw Angle: {yaw_angle}, Degrees: {np.rad2deg(yaw_angle)}")

#         # Adjust the loop rate as needed
#         rospy.sleep(1.0)
