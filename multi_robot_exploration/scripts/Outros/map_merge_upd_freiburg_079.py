#!/usr/bin/python3

import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rospy
import time
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import tf


class MapMerge:
    def __init__(self):

        rospy.init_node('map_merde_upd', anonymous=True)

        rospy.Subscriber('/map_merge', OccupancyGrid, self.map_callback)

        self.pub = rospy.Publisher('/map_merge_upd', OccupancyGrid, queue_size=10)

        self.last_grid = 0

    def map_callback(self, map_merge: OccupancyGrid):

        # Get the map dimensions and resolution
        width = map_merge.info.width
        height = map_merge.info.height
        resolution = map_merge.info.resolution

        data = list(map_merge.data)

        print(len(map_merge.data))

        if len(map_merge.data) >= 255642:

            if self.last_grid != 0:
                for y in range(height):
                    for x in range(width):

                        i = x + (height - 1 - y) * width

                        if self.last_grid[i] == 0 and data[i] != 0:
                        
                            data[i] = self.last_grid[i]

            # Save the last map
            self.last_grid = data

            # Compute properly grid to publish map
            map_merge.data = tuple(data)

            # Publish /map_merge_upd
            self.pub.publish(map_merge)
    

if __name__ == '__main__':

    frontier_extractor = MapMerge()
    rospy.spin()