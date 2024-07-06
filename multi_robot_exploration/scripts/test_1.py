#!/usr/bin/python3

import rospy
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import PointStamped
import numpy as np
import math
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
import tf.msg
import tf2_ros
import tf.transformations as tf_trans
import tf2_msgs.msg
from sklearn.cluster import DBSCAN
from geometry_msgs.msg import Point
from first_robot_navigation.msg import Goal_msg
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs


class MultiFrontierExploration:
    def __init__(self):
        # ROS node initialization
        rospy.init_node('multi_frontier_exploration')

        # List to store frontier points
        self.frontier_points = []  

        # Robot positions
        self.robot_pose = None

        # Robot prefix
        self.robot_namespace = rospy.get_param('~robot_namespace')

        # Create a TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Subscribe to TF topic for each robot
        self.tf_sub_robot = rospy.Subscriber("/tf", tf2_msgs.msg.TFMessage, self.tf_callback_robot)

        # Subscribe to the occupancy grid map
        rospy.Subscriber('/map_merge_upd', OccupancyGrid, self.map_callback)

        # Publisher for frontier points as markers
        self.marker_publisher = rospy.Publisher('cluster_markers', Marker, queue_size=10)

        # Publisher for MoveBaseGoal messages
        self.move_base_pub = rospy.Publisher('/'+ self.robot_namespace +'/move_base/goal', MoveBaseActionGoal, queue_size=10)

#---------------------------------------------------------------------------------------------------------------------------------------------------

        self.goal = None

        self.robot_goals = []  # Dictionary to store the goal points of all robots

        # Publisher for publishing goal point
        self.goal_publisher = rospy.Publisher('/goal_point', Goal_msg, queue_size=10)

        # Subscriber to see goal points of other robots
        rospy.Subscriber('/goal_point', Goal_msg, self.goal_point_callback)

        self.marker_pub = rospy.Publisher('point_markers', Marker, queue_size=10)
    
    
    def map_callback(self, msg):

        # Perform frontier extraction and delete points close to walls on the received map
        self.extract_frontiers(msg)
     
        # Transform frontier points into clusters
        epsilon = 0.2
        min_samples = 7
        cluster_points = self.cluster_frontier_points(self.frontier_points, epsilon, min_samples)

        # Remove wrong clusters
        #cluster_points = [points for points in cluster_points if math.sqrt((points[-1][0] - points[0][0]) ** 2 + (points[-1][1] - points[0][1]) ** 2) <= 4.5]
   
        # Visualize the clusters
        self.visualize_clusters(msg, cluster_points)

        # calculate de size and middle point of frontier
        frontier_robot = self.calculate_frontier_size(cluster_points)

        # Calculate Distance of frontier
        frontier_robot = self.calculate_distance_frontier(frontier_robot)

        # Select best frontier point
        goal_point = self.select_best_frontier(frontier_robot)

        # Task allocation
        if self.check_goal_point(goal_point, self.robot_namespace):
            # Delete the frontier point where goal point exist  
            frontier_robot = [data for data in frontier_robot if data[2].point.x != goal_point.point.x or data[2].point.y != goal_point.point.y]

        goal_point = self.select_best_frontier(frontier_robot)  

        #self.initial_entropy = self.calc_entropy(msg)
        #print(self.initial_entropy)

        # Send the robot to goal point
        self.send_goal(goal_point)

        # Publish robot goal
        self.publish_goal(goal_point, self.robot_namespace)



    def extract_frontiers(self, occupancy_grid):
        frontiers = []

        # Get the map dimensions and resolution
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        resolution = occupancy_grid.info.resolution

        # Convert the occupancy grid data to a 2D matrix
        grid = [[0] * width for _ in range(height)]
        for i in range(height):
            for j in range(width):
                index = j + width * i
                grid[i][j] = occupancy_grid.data[index]
        
        # Iterate over the grid to find frontier points
        for i in range(height):
            for j in range(width):
                if grid[i][j] >= 0 and grid[i][j] <= 70:  # Unoccupied cell
                    neighbors = self.get_neighbors(grid, i, j)
                    for neighbor in neighbors:
                        if grid[neighbor[0]][neighbor[1]] == -1:  # Unknown cell
                            frontiers.append((i, j))
                            break

        # Convert frontier points to PointStamped messages
        self.frontier_points = []
        for frontier in frontiers:
            x = (frontier[1] + 0.5) * resolution + occupancy_grid.info.origin.position.x
            y = (frontier[0] + 0.5) * resolution + occupancy_grid.info.origin.position.y
            point = PointStamped()
            point.point.x = x
            point.point.y = y
            point.point.z = 0.0
            self.frontier_points.append(point)
        #----------------------------------------------------------------------------------------------------------------
        # Filter frontiers points close to walls! Check if there's a occ cell on a 6x6 grid around the frontier point
        filtered_points = []
        for point in self.frontier_points:
            # Convert frontier point coordinates to grid indices
            grid_x = int((point.point.x - occupancy_grid.info.origin.position.x) / occupancy_grid.info.resolution)
            grid_y = int((point.point.y - occupancy_grid.info.origin.position.y) / occupancy_grid.info.resolution)

            # Check if the point is close to an occupied cell in the occupancy grid
            close_to_wall = False
            for dx in range(-2, 4):  # Modified range for x-axis (-2 to 3)
                for dy in range(-2, 4):  # Modified range for y-axis (-2 to 3)
                    neighbor_x = grid_x + dx
                    neighbor_y = grid_y + dy

                    # Check if the neighbor cell is an occupied cell (value = 100 in the occupancy grid)
                    if 0 <= neighbor_x < occupancy_grid.info.width and 0 <= neighbor_y < occupancy_grid.info.height:
                        cell_index = neighbor_y * occupancy_grid.info.width + neighbor_x
                        if occupancy_grid.data[cell_index] >= 80:
                            close_to_wall = True
                            break

                if close_to_wall:
                    break

            # If the point is not close to a wall, add it to the filtered points
            if not close_to_wall:
                filtered_points.append(point)

        self.frontier_points = filtered_points
    #----------------------------------------------------------------------------------------------------------------

    def cluster_frontier_points(self, frontier_points, epsilon, min_samples):
        # Extract numerical coordinates from PointStamped objects
        points = [[point.point.x, point.point.y] for point in frontier_points]

        # Convert points to a NumPy array
        points = np.array(points)

        # Perform DBSCAN clustering
        dbscan = DBSCAN(eps=epsilon, min_samples=min_samples)
        labels = dbscan.fit_predict(points)

        # Collect points for each cluster
        clusters = {}
        for i, label in enumerate(labels):
            if label not in clusters:
                clusters[label] = []
            clusters[label].append(points[i])

        # Return cluster points
        return list(clusters.values())
    #----------------------------------------------------------------------------------------------------------------
    def calculate_frontier_size(self, cluster_points):
        table = []

        for points in cluster_points:

            # Compute information gain for each cluster
            num_fp = len(points)
            if num_fp > 20:

                # Determine the middle point or the point next to the one in the middle
                middle_index = len(points) // 2
                if len(points) % 2 == 0:
                    middle_point = points[middle_index - 1]
                else:
                    middle_point = points[middle_index]

                # Add the size and middle point to the result list
                x=middle_point[0]
                y=middle_point[1]
                middle_point = PointStamped()
                middle_point.point.x = x
                middle_point.point.y = y
                middle_point.point.z = 0.0
                
                points = np.array(points)

            
                table.append((num_fp, middle_point))

        return table
    #----------------------------------------------------------------------------------------------------------------
    def compute_density(self, frontier_robot, map_type):
        # Atualiza o frontier density do goal point
        table=[]
        frontier_density = 0
        for data in frontier_robot:
            size, middle_point = data
            frontier_density = 0

            for data_1 in frontier_robot:
                size_1, middle_point_1 = data_1

                size = 0.0017 
                table.append((size, middle_point))

        return table

    #----------------------------------------------------------------------------------------------------------------
   
    def get_neighbors(self, grid, i, j):
        neighbors = []
        height = len(grid)
        width = len(grid[0])

        if i > 0:
            neighbors.append((i - 1, j))  # Upper neighbor
        if i < height - 1:
            neighbors.append((i + 1, j))  # Lower neighbor
        if j > 0:
            neighbors.append((i, j - 1))  # Left neighbor
        if j < width - 1:
            neighbors.append((i, j + 1))  # Right neighbor

        return neighbors
    
    def send_goal(self, point):
        goal_msg = MoveBaseActionGoal()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.goal.target_pose.header.frame_id = "map_merged"
        goal_msg.goal.target_pose.pose.position.x = point.point.x
        goal_msg.goal.target_pose.pose.position.y = point.point.y
        goal_msg.goal.target_pose.pose.orientation.w = 1.0
        self.move_base_pub.publish(goal_msg)

    def tf_callback_robot(self, tf_msg):
        try:
            # Lookup the transform between the map frame and the robot's base frame
            transform = self.tf_buffer.lookup_transform("map_merged", self.robot_namespace + "/base_link", rospy.Time())

            # Extract the position from the transform
            self.robot_pose = transform.transform.translation

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to retrieve transform for robot")
    
    def calculate_distance(self, point1, point2):
        if point1 is None or point2 is None:
            rospy.logwarn("One or both points are None. Cannot calculate distance.")
            return None
        # Extract x and y coordinates from PointStamped objects
        x1 = point1.point.x
        y1 = point1.point.y
        x2 = point2.x
        y2 = point2.y

        # Calculate Euclidean distance between the points
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return distance
    
    def visualize_clusters(self, msg, cluster_points):
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id
        marker.type = Marker.POINTS  # Change the marker type to POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2  # Point size, adjust as needed
        marker.scale.y = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        for points in cluster_points:
            for point in points:
                # Add each point as a separate marker
                marker.points.append(Point(point[0], point[1], 0.0))

        self.marker_publisher.publish(marker)

    def select_best_frontier(self,frontier_robot):
        # Send the robot to the bigest frontier
        min_distance = +float('inf')  # Initialize with negative infinity
        min_middle_point = None

        for data in frontier_robot:
            size, distance, middle_point = data
            if distance < min_distance:
                min_distance = distance
                min_middle_point = middle_point
        
        return min_middle_point

    def calculate_distance_frontier(self, table):
        frontier_robot =[]
        for size, middle_point in table:
            # Calculate distances to robot's position
            distance_robot = self.calculate_distance(middle_point, self.robot_pose)
            
            # Create a tuple or list with size and middle_point
            frontier_data = (size, distance_robot, middle_point)

            # Add distance information
            frontier_robot.append(frontier_data)

        return frontier_robot

    def goal_point_callback(self, data):
        # Extract the relevant information from the received message
        robot_id = data.robot_id
        goal_x = data.point.x
        goal_y = data.point.y

        # Check if the robot ID already exists in the robot_goals table
        for i, goal in enumerate(self.robot_goals):
            if goal[0] == robot_id:
                # Update the existing entry with the new goal point
                self.robot_goals[i] = (robot_id, goal_x, goal_y)
                break
        else:
            # If the robot ID doesn't exist, append the new goal point
            self.robot_goals.append((robot_id, goal_x, goal_y))

        #for goal in self.robot_goals:
        #    print("Robot ID:", goal[0], goal[1], goal[2])
            
        #print("---------------------------------------------------------------------------------")  # Add an empty line between entries

    def check_goal_point(self, goal_point, namespace):
        threshold = 1
        for goal in self.robot_goals:
            x_diff = goal[1] - goal_point.point.x
            y_diff = goal[2] - goal_point.point.y
            distance = math.sqrt(x_diff**2 + y_diff**2)

            if goal[0] != namespace and distance <= threshold:
                return True
        return False

    def publish_goal(self, goal_point, namespace):
        # Create a CustomGoal message
        custom_goal = Goal_msg()
        custom_goal.robot_id = namespace

        # Assign the Point instance to the custom_goal's goal_point field
        custom_goal.point.x = goal_point.point.x
        custom_goal.point.y = goal_point.point.y
        
        #print ("Goal Point = {} {}".format(goal_point.point.x,goal_point.point.y))

        # Publish the goal message
        self.goal_publisher.publish(custom_goal)

    def send_goal_point(self):
        # Publish the goal point to the topic
        rospy.Publisher('goal_point', Point, queue_size=10).publish(self.goal_point)

    def calc_entropy(self, occupancy_grid):

        entropy = 0
        for cell in occupancy_grid.data:
            if cell == -1:
                entropy -= -1.0
            if cell > 0 and cell < 100:
                p_cell = cell/100
                entropy -= p_cell*math.log2(p_cell) + (1-p_cell)*math.log2(1-p_cell)

        entropy = entropy/len(occupancy_grid.data)

        return entropy

    
def filter_frontier_points(frontier_points, threshold_distance):
    # Initialize a list to store the filtered frontier points
    filtered_points = []

    # Iterate over each frontier point
    while frontier_points:
        # Take the first point as a reference
        reference_point = frontier_points.pop(0)
        reference_x, reference_y = reference_point.point.x, reference_point.point.y

        # Iterate over the remaining frontier points
        i = 0
        while i < len(frontier_points):
            # Calculate the Euclidean distance between the reference point and the current point
            current_x, current_y = frontier_points[i].point.x, frontier_points[i].point.y
            distance = math.sqrt((current_x - reference_x) ** 2 + (current_y - reference_y) ** 2)

            # If the distance is below the threshold, remove the current point
            if distance <= threshold_distance:
                frontier_points.pop(i)
            else:
                i += 1

        # Add the reference point to the filtered points
        filtered_points.append(reference_point)

    return filtered_points

if __name__ == '__main__':

    frontier_extractor = MultiFrontierExploration()
    rospy.spin()