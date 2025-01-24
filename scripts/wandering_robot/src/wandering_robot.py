#!/usr/bin/env python

# ROS imports
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Callback function to process LaserScan data
def callback(msg):
    global move
    safe_distance = 0.5  # Minimum safe distance from obstacles

    # Check distance directly in front of the robot
    if msg.ranges[0] > safe_distance:
        # No obstacle ahead: Move forward
        move.linear.x = 0.5
        move.angular.z = 0.0
    else:
        # Obstacle ahead: Reverse slightly and turn
        move.linear.x = -0.2
        move.angular.z = 0.5

    # Publish the movement command
    pub.publish(move)

# Main function
if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)

    # Publisher to control the robot's movement
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Subscriber to the LaserScan topic
    sub = rospy.Subscriber('/scan', LaserScan, callback)

    # Initialize the Twist message
    move = Twist()

    rospy.spin()
