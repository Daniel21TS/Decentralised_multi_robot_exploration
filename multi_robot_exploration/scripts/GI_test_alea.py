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
from multi_robot_exploration.msg import Goal_msg
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs
import pandas as pd
import os
import random
#from openpyxl import load_workbook



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

        #self.goal_point = Point()

        self.goal = None
        self.last_goal_point = None

        self.last_goal_point_1 = 0

        self.info_tab = []

        self.robot_goals = []  # Dictionary to store the goal points of all robots

        # Publisher for publishing goal point
        self.goal_publisher = rospy.Publisher('/goal_point', Goal_msg, queue_size=10)

        # Subscriber to see goal points of other robots
        rospy.Subscriber('/goal_point', Goal_msg, self.goal_point_callback)

        self.marker_pub = rospy.Publisher('point_markers', Marker, queue_size=10)

        self.distance_trav = 0
        self.last_robot_pose = self.robot_pose
        self.a = 0
    
    
    def map_callback(self, msg):

        # Perform frontier extraction and delete points close to walls on the received map
        self.extract_frontiers(msg)
     
        # Transform frontier points into clusters
        epsilon = 0.1
        min_samples = 1
        cluster_points = self.cluster_frontier_points(self.frontier_points, epsilon, min_samples)
   
        # Visualize the clusters
        self.visualize_clusters(msg, cluster_points)

        # Calculate number of frontier points for each frontier and respective information gain and the middle point
        map_type = 1
        resolution = 0.045
        frontier_robot = self.calculate_frontier_size(cluster_points, resolution)

        # Calculate de density of interest points and choose the respective information gain
        frontier_robot = self.compute_density(frontier_robot)

        # Calculate Distance of frontier
        frontier_robot = self.calculate_distance_frontier(frontier_robot)

        # Implement adaptative exploration area
        frontier_robot = self.calculate_adaptative_exploration_area(frontier_robot)

        # Calculate Score with Objective Function
        #wD=0.35
        #wS=0.45
        #wDen=0.3
        #frontier_robot = self.calculate_score(frontier_robot,wD,wS,wDen)

        print("Possible frontiers for " + self.robot_namespace + ":")
        for data in frontier_robot:
            size, dens, distance, middle_point = data
            print("size={}, distance={}, density={}, x={}, y={} ".format(size, distance, dens, middle_point.point.x , middle_point.point.y ))

        entropy = self.calc_entropy(msg)
        print("Actual Entropy = {}".format( entropy))

        check = 0
        if self.last_goal_point_1 == 0:
            goal_point = self.select_best_frontier(frontier_robot)
        else:
            #check if goal_point exist in frontier_robot
            for data in frontier_robot:
                size, dens, distance, middle_point = data
                if middle_point.point.x == self.last_goal_point_1.point.x:
                    check = 1

            if check == 1:
                goal_point = self.last_goal_point_1
            else:
                goal_point = self.select_best_frontier(frontier_robot)
            
        check = 0
        self.last_goal_point_1 = goal_point

        # Send the robot to goal point
        self.send_goal(goal_point)

#-----------------------------------------------------------------------------------------------------------
        print("x goal point = ",goal_point.point.x)
        print("y goal point = ",goal_point.point.y)

        #print("Frontier " + self.robot_namespace + ":")
        #for data in frontier_robot:
        #    size, distance, middle_point, score = data
        #    if goal_point.point.x == middle_point.point.x:
        #        print("size={}, distance={}, x={}, y={}, score={}".format(size, distance, middle_point.point.x , middle_point.point.y, score))

        if self.last_goal_point is None:
            self.initial_entropy = self.calc_entropy(msg)
            self.last_goal_point = goal_point

            # Compute frontier size for first goal point
            self.size_last_frontier = 0
            self.densidade_ponderada = 0
            for data in frontier_robot:
                size, dens, distance, middle_point = data
                if self.last_goal_point.point.x == middle_point.point.x:
                    self.size_last_frontier = size
                    self.densidade_ponderada = dens
        
        elif abs(goal_point.point.x - self.last_goal_point.point.x) > 0.5:

            # Calcular a entropia final após o goal_point mudar (se necessário)
            final_entropy = self.calc_entropy(msg) 

            # Calcular o ganho de informação
            information_gain = self.initial_entropy - final_entropy

            norm_info_gain = information_gain / 5.6

            data_zz = (self.last_goal_point, self.size_last_frontier, map_type, norm_info_gain, self.densidade_ponderada)

            self.info_tab.append(data_zz)

            print("Numero de fronteiras visitadas = {}".format(len(self.info_tab) ))

            #Colocar no ficheiro
            if len((self.info_tab)) >= 50:
                data = self.info_tab
                df = pd.DataFrame(data)
                output_file = "/home/daniel21/Robot_ws/src/output.xlsx"
                df.to_excel(output_file, index=False)

            # Atualiza entropia inicial
            self.initial_entropy = self.calc_entropy(msg)
            self.last_goal_point = goal_point

            # Atualiza o size do ultimo goal point
            for data in frontier_robot:
                size, dens, distance, middle_point = data
                if self.last_goal_point.point.x == middle_point.point.x:
                    self.size_last_frontier = size
                    self.densidade_ponderada = dens

            print("tabela de fronteiras visitadas :")
            for data in self.info_tab:
                last_goal_point, size_last_frontier, map_type, information_gain, frontier_density_ponderada = data
                print("x={}, y={}, Frontier Size={}, Map Type={}, Norm information Gain={}, Frontier Density Ponderada={}".format(last_goal_point.point.x , last_goal_point.point.y, size_last_frontier, map_type, information_gain, frontier_density_ponderada))        

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
    
    #def cluster_frontier_points(self, frontier_points, epsilon, min_samples):
        # Extract numerical coordinates from PointStamped objects
    #    points = [[point.point.x, point.point.y] for point in frontier_points]

        # Convert points to a NumPy array
    #    points = np.array(points)

        # Perform DBSCAN clustering
    #    dbscan = DBSCAN(eps=epsilon, min_samples=min_samples)
    #    labels = dbscan.fit_predict(points)

        # Check if there is only one large cluster
    #    unique_labels, label_counts = np.unique(labels, return_counts=True)

    #    if label_counts > 35:
            # There is only one cluster with more than 20 points, split it
    #        large_cluster_label = unique_labels[0]
    #        large_cluster_indices = np.where(labels == large_cluster_label)[0]

            # Split the large cluster into smaller clusters of 20 points each
    #        cluster_indices = np.array_split(large_cluster_indices, label_counts[0] // 20)

            # Create new labels for the split clusters
    #        new_labels = np.copy(labels)
    #        for i, indices in enumerate(cluster_indices):
    #            new_labels[indices] = max(unique_labels) + 1 + i

    #        labels = new_labels

        # Collect points for each cluster
    #    clusters = {}
    #    for i, label in enumerate(labels):
    #        if label not in clusters:
    #            clusters[label] = []
    #        clusters[label].append(points[i])

        # Return cluster points
    #    return list(clusters.values())

    #----------------------------------------------------------------------------------------------------------------
    def calculate_frontier_size(self, cluster_points, resolution):
        table = []

        # se o mapa for deste tipo, o "info_gain_per_cell" vai ser x. Se for uma curva adiciona y ao "info_gain_per_cell", se for reta adiciona z,...

        for points in cluster_points:

            # Compute information gain for each cluster
            num_fp = len(points)
            if num_fp > 8:

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

                size_frontier = num_fp * resolution

                #print(size_frontier)

                table.append((size_frontier, middle_point))

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
    
    def calculate_distance_1(self, point1, point2):
        if point1 is None or point2 is None:
            rospy.logwarn("One or both points are None. Cannot calculate distance.")
            return None
        # Extract x and y coordinates from PointStamped objects
        x1 = point1.x
        y1 = point1.y
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
            #print( len(points) )
            if len(points) > 8:
                for point in points:
                    # Add each point as a separate marker
                    marker.points.append(Point(point[0], point[1], 0.0))

        self.marker_publisher.publish(marker)


    #def select_best_frontier(self,frontier_robot):
        # Send the robot to the bigest frontier
    #    max_score = -float('inf')  # Initialize with negative infinity
    #    max_middle_point = None

    #    for data in frontier_robot:
    #        size, dens, distance, middle_point, score = data
    #        if score > max_score:
    #            max_score = score
    #            max_middle_point = middle_point
        
    #    return max_middle_point

    def select_best_frontier(self,frontier_robot):

        # Crie uma lista de pontos médios das fronteiras
        middle_points = [data[3] for data in frontier_robot]

        # Escolha aleatoriamente um ponto médio das fronteiras
        random_middle_point = random.choice(middle_points)
        
        return random_middle_point

    def calculate_distance_frontier(self, table):
        frontier_robot =[]
        for size, dens, middle_point in table:
            # Calculate distances to robot's position
            distance_robot = self.calculate_distance(middle_point, self.robot_pose)
            
            # Create a tuple or list with size and middle_point
            frontier_data = (size, dens, distance_robot, middle_point)

            # Add distance information
            frontier_robot.append(frontier_data)

        return frontier_robot
    
    def calculate_adaptative_exploration_area(self, table):

        frontier_robot =[]
        raio = 5

        # while frontier robot vazia ou fronteiras fracas
        while len(frontier_robot) == 0 :
            for size, dens, distance_robot, middle_point in table:
                
                if distance_robot < raio and size > 2 and dens > 0.05:
                
                    # Create a tuple or list with size and middle_point
                    frontier_data = (size, dens, distance_robot, middle_point)

                    # Add distance information
                    frontier_robot.append(frontier_data)

            #if variavel == False
            #print("Raio de exploraçao: ")
            #print(raio)
            raio = raio + 2
            
        return frontier_robot

    def compute_density(self, frontier_robot):

        frontier_robot_new =[]
        frontier_density = 0
        for data in frontier_robot:
            size, middle_point = data
            
            frontier_density = 0
            area_field_of_view = 65.68
            tamanho_fronteiras_vizinhas = 0
            for data_1 in frontier_robot:
                size_1, middle_point_1 = data_1

                if abs(middle_point.point.x - middle_point_1.point.x) < 5.6 and abs(middle_point.point.y - middle_point_1.point.y) < 5.6:
                    frontier_density += 1
                    tamanho_fronteiras_vizinhas += size_1


            density = frontier_density/area_field_of_view
            media_fronteiras_vizinhas = tamanho_fronteiras_vizinhas / frontier_density
            densidade_ponderada = density * media_fronteiras_vizinhas / size

            # Create a tuple or list with size and middle_point
            frontier_data = (size, densidade_ponderada, middle_point)

            # Add distance information
            frontier_robot_new.append(frontier_data)

        return frontier_robot_new
    
    def calculate_score(self, frontier_robot, wD,wS,wDen):
        new_frontier_robot = []

        # Extract the values for each parameter
        sizes = [entry[0] for entry in frontier_robot]
        dens = [entry[1] for entry in frontier_robot]
        distances = [entry[2] for entry in frontier_robot]

        # Calculate the mean and standard deviation for each parameter
        mean_size = np.mean(sizes)
        std_size = np.std(sizes)

        mean_dens = np.mean(dens)
        std_dens = np.std(dens)

        mean_distance = np.mean(distances)
        std_distance = np.std(distances)

        for size, dens, distance, middle_point in frontier_robot:

            # Calculate normalized values
  
            if std_size == 0:
                normalized_size = 1

            else:
                normalized_size = (size - mean_size) / std_size

            if std_distance == 0:
                normalized_distance = 1
            else:
                normalized_distance = (distance - mean_distance) / std_distance

            if std_dens == 0:
                normalized_dens = 1
            else:
                normalized_dens = (dens - mean_dens) / std_dens

            score = wS*normalized_size + wDen*normalized_dens - wD*normalized_distance
            new_frontier_robot.append((size, dens, distance, middle_point, score))

        frontier_robot = new_frontier_robot

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

    #def calc_entropy(self, occupancy_grid):

    #    entropy = 0
    #    for cell in occupancy_grid.data:
    #        if cell == -1:
    #            entropy -= -1.0
    #        if cell > 0 and cell < 100:
    #            p_cell = cell/100
    #            entropy -= p_cell*math.log2(p_cell) + (1-p_cell)*math.log2(1-p_cell)

    #    entropy = entropy/len(occupancy_grid.data)

    #    entropy = entropy-0.5144
    #    entropy = entropy/0.4856

    #    return entropy
    
    def calc_entropy(self, occupancy_grid):

        entropy = 0
        for cell in occupancy_grid.data:
            if cell == -1:
                entropy -= -1.0
            if cell > 0 and cell < 100:
                p_cell = cell/100
                entropy -= p_cell*math.log2(p_cell) + (1-p_cell)*math.log2(1-p_cell)

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