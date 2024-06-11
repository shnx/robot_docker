#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time

class MovementReplayer:
    def __init__(self, filename='recorded_movements.txt'):
        rospy.init_node('movement_replayer', anonymous=True)
        self.movement_publisher = rospy.Publisher('/pioneer/cmd_vel', Twist, queue_size=10)
        self.movements = self.load_movements(filename)
        self.movements_generator = self.generate_movements()

    def load_movements(self, filename):
        movements = []
        with open(filename, 'r') as file:
            for line in file:
                linear, angular = map(float, line.split())
                twist = Twist()
                twist.linear.x = linear
                twist.angular.z = angular
                movements.append(twist)
        return movements

    def generate_movements(self):
        for twist in self.movements:
            yield twist

    def next_move(self):
        try:
            twist = next(self.movements_generator)
            self.movement_publisher.publish(twist)
            time.sleep(0.1)  # Adjust sleep duration based on your robot's response time
        except StopIteration:
            rospy.loginfo("All movements replayed")

if __name__ == '__main__':
     replayer = MovementReplayer()

     # Replay the recorded movements step by step
     for _ in range(len(replayer.movements)):
         replayer.next_move()
        # Perform any additional logic or wait as needed between movements
