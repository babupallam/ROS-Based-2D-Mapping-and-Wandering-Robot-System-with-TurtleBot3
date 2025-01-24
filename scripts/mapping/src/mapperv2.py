#!/usr/bin/env python
""" Simple occupancy-grid-based mapping without localization. 

Subscribed topics:
/scan

Published topics:
/map 
/map_metadata

"""
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Map(object):
    """ 
    The Map class stores an occupancy grid as a two dimensional
    numpy array. 
    
    Public instance variables:

        width      --  Number of columns in the occupancy grid.
        height     --  Number of rows in the occupancy grid.
        resolution --  Width of each grid square in meters. 
        origin_x   --  Position of the grid cell (0,0) in 
        origin_y   --    in the map coordinate system.
        grid       --  numpy array with height rows and width columns.
        
    
    Note that x increases with increasing column number and y increases
    with increasing row number. 
    """

    #def __init__(self, origin_x=-2.5, origin_y=-2.5, resolution=.1, width=50, height=50):
    #def __init__(self, origin_x=-2.5, origin_y=-2.5, resolution=.1, width=50, height=50):
    def __init__(self, origin_x=-5, origin_y=-5, resolution=.1, width=100, height=100):
        """ Construct an empty occupancy grid.
        
        Arguments: origin_x, 
                   origin_y  -- The position of grid cell (0,0) in the
                                map coordinate frame.
                   resolution-- width and height of the grid cells 
                                in meters.
                   width, 
                   height    -- The grid will have height rows and width
                                columns cells.  width is the size of
                                the x-dimension and height is the size
                                of the y-dimension.
                                
         The default arguments put (0,0) in the center of the grid. 
                                
        """
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.width = width 
        self.height = height 
        self.grid = np.zeros((height, width))

    def to_message(self):
        """ Return a nav_msgs/OccupancyGrid representation of this map. """
     
        grid_msg = OccupancyGrid()

        # Set up the header.
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"

        # .info is a nav_msgs/MapMetaData message. 
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height
        
        # Rotated maps are not supported... quaternion represents no
        # rotation. 
        grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0),
                               Quaternion(0, 0, 0, 1))

        # Flatten the numpy array into a list of integers from 0-100.
        # This assumes that the grid entries are probalities in the
        # range 0-1. This code will need to be modified if the grid
        # entries are given a different interpretation (like
        # log-odds).
        

        flat_grid = self.grid.reshape((self.grid.size,)) * 100
        #print('-----------')
        
        #print (flat_grid)
        flat_grid=flat_grid.astype('int8')
      
        ##########


        grid_msg.data = list(np.round(flat_grid))
       

        return grid_msg

    def set_cell(self, x, y, val):
        """Set the value of a cell in the grid."""
        # Convert from global coordinates to grid indices
        i = int((x - self.origin_x) / self.resolution)
        j = int((y - self.origin_y) / self.resolution)

        # Ensure indices are within bounds
        if 0 <= i < self.width and 0 <= j < self.height:
            self.grid[j, i] = val
        else:
            rospy.logdebug(f"Coordinates ({x:.2f}, {y:.2f}) out of grid bounds.")

    def get_index(self, x, y):
        """Convert global coordinates to grid indices."""
        i = int((x - self.origin_x) / self.resolution)
        j = int((y - self.origin_y) / self.resolution)
        return i, j



class Mapper(object):
    """ 
    The Mapper class creates a map from laser scan data.
    """
    
    def __init__(self):
        """ Start the mapper. """

        rospy.init_node('mapper')
        self._map = Map()

        # Setting the queue_size to 1 will prevent the subscriber from
        # buffering scan messages.  This is important because the
        # callback is likely to be too slow to keep up with the scan
        # messages. If we buffer those messages we will fall behind
        # and end up processing really old scans.  Better to just drop
        # old scans and always work with the most recent available.
        rospy.Subscriber('scan',
                         LaserScan, self.scan_callback, queue_size=1)

        # Latched publishers are used for slow changing topics like
        # maps.  Data will sit on the topic until someone reads it. 
        self._map_pub = rospy.Publisher('map', OccupancyGrid, latch=True)
        self._map_data_pub = rospy.Publisher('map_metadata', 
                                             MapMetaData, latch=True)
        
        rospy.spin()

    def get_robot_position(self):
        """Fetch the robot's position and orientation in the global frame."""
        try:
            odom_msg = rospy.wait_for_message('/odom', Odometry, timeout=1)
            position = odom_msg.pose.pose.position
            orientation = odom_msg.pose.pose.orientation
            roll, pitch, yaw = euler_from_quaternion(
                [orientation.x, orientation.y, orientation.z, orientation.w]
            )
            return position.x, position.y, yaw
        except rospy.ROSException as e:
            rospy.logwarn("Failed to fetch robot position: %s", str(e))
            return 0.0, 0.0, 0.0


    def scan_callback(self, scan):
        """Update the map based on laser scan data."""
        rospy.loginfo("Processing laser scan data...")

        # Get the robot's position and orientation in the global frame
        robot_x, robot_y, robot_yaw = self.get_robot_position()

        for angle_index in range(len(scan.ranges)):
            distance = scan.ranges[angle_index]

            # Filter out invalid or noisy readings
            if distance < scan.range_min or distance > scan.range_max:
                continue
            if angle_index > 0:
                previous_distance = scan.ranges[angle_index - 1]
                if abs(distance - previous_distance) > 0.5:  # Noise threshold
                    continue

            # Calculate the angle of the current laser scan reading
            angle = scan.angle_min + angle_index * scan.angle_increment

            # Convert polar coordinates (distance, angle) to local Cartesian coordinates
            local_x = distance * math.cos(angle)
            local_y = distance * math.sin(angle)

            # Transform local coordinates to global coordinates
            global_x = local_x * math.cos(robot_yaw) - local_y * math.sin(robot_yaw) + robot_x
            global_y = local_x * math.sin(robot_yaw) + local_y * math.cos(robot_yaw) + robot_y

            # Use get_index to map global coordinates to grid indices
            i, j = self._map.get_index(global_x, global_y)

            # Mark the grid cell as occupied (1)
            if 0 <= i < self._map.width and 0 <= j < self._map.height:
                self._map.grid[j, i] = 1

            # Mark cells between the robot and the obstacle as free space
            for step in range(int(distance / self._map.resolution)):
                free_x = robot_x + step * self._map.resolution * math.cos(angle + robot_yaw)
                free_y = robot_y + step * self._map.resolution * math.sin(angle + robot_yaw)

                # Map free space coordinates to grid indices
                free_i, free_j = self._map.get_index(free_x, free_y)
                if 0 <= free_i < self._map.width and 0 <= free_j < self._map.height:
                    self._map.grid[free_j, free_i] = 0

        # Publish the updated map
        self.publish_map()



    def publish_map(self):
        """ Publish the map. """
        grid_msg = self._map.to_message()
        self._map_data_pub.publish(grid_msg.info)
        self._map_pub.publish(grid_msg)


if __name__ == '__main__':
    try:
        m = Mapper()
    except rospy.ROSInterruptException:
        pass

"""
https://w3.cs.jmu.edu/spragunr/CS354_S14/labs/mapping/mapper.py
https://www.youtube.com/watch?v=K1ZFkR4YsRQ
"""