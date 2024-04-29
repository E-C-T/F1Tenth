#!/usr/bin/env python3

"""
This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""
import numpy as np
from numpy import linalg as LA
import math
import random
import time
from scipy.interpolate import splprep, splev
 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import OccupancyGrid, Path
import csv
from visualization_msgs.msg import Marker, MarkerArray

# class def for tree nodes
class node(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = []
        self.cost = None # only used in RRT*
        self.is_root = False

# class def for RRT
class RRT(Node):
    def __init__(self):
        super().__init__('rrt_node')
        # topics, not saved as attributes
        # grab topics from param file, you'll need to change the yaml file
        pose_topic = "ego_racecar/odom"
        scan_topic = "/scan"

        # create subscribers
        self.scan_sub_ = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            1)
        self.scan_sub_
        self.pose_sub_ = self.create_subscription(
            Odometry, 
            pose_topic, 
            self.pose_callback, 
            1)
        self.pose_sub_
       

        # publishers
        # create a drive message publisher, and other publishers that you might need
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.opp_drive_pub = self.create_publisher(AckermannDriveStamped, '/opp_drive', 10)

        self.grid_pub_ = self.create_publisher(OccupancyGrid, '/map/grid', 10)
        self.path_pub = self.create_publisher(Path, '/path/rrt_path', 10)
        self.target_point_pub = self.create_publisher(Marker, '/point/target', 10)
        self.follow_point_pub = self.create_publisher(Marker, '/point/follow', 10)
        self.tree_pub = self.create_publisher(Marker, 'rrt_tree', 10)
        # class attributes
        # create occupancy grid, waypoints csv must be updated for map 
        csv_file_path = './src/f1tenth_gym_ros/rcws/logs/wp-2023-10-14-23-51-55.csv' # slam map

        waypoint_list = []
        with open(csv_file_path, 'r') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                if len(row) >= 2:
                    waypoint_list.append([float(row[0]), float(row[1])])

        self.waypoints = np.array(waypoint_list)  # Define your own waypoints
        _, ind = np.unique(self.waypoints, axis=0, return_index=True)
        ind = np.sort(ind)
        self.waypoints = self.waypoints[ind]
        self.lookahead = 1.5 
        self.last_idx = 0
        
        # occupancy grid params
        self.resolution = 0.1 #0.1    
        self.map_width = 60 
        self.map_height = 40
        self.grid_offset = float(-(self.map_height*self.resolution / 2))

        self.occupancy_map = OccupancyGrid()
        self.occupancy_map.info.width = self.map_width #cells
        self.occupancy_map.info.height = self.map_height #cells
        self.occupancy_map.info.resolution = self.resolution # m/cell
        self.occupancy_map.info.origin.position.x = 0.0
        self.occupancy_map.info.origin.position.y = self.grid_offset
        self.occupancy_map.data = [0] * (self.map_width * self.map_height)

    def get_oc_grid(self, scan_msg):
        self.occupancy_map = OccupancyGrid()
        self.occupancy_map.header = scan_msg.header
        self.occupancy_map.info.width = self.map_width #cells
        self.occupancy_map.info.height = self.map_height #cells
        self.occupancy_map.info.resolution = self.resolution # m/cell
        self.occupancy_map.info.origin.position.x = 0.0
        self.occupancy_map.info.origin.position.y = self.grid_offset
        grid = [0] * (self.map_height * self.map_width)

        # Process LIDAR data and update occupancy grid
        for i, range_measurement in enumerate(scan_msg.ranges):
            if range_measurement < scan_msg.range_max:
                grid_x= int(( range_measurement * np.cos( scan_msg.angle_min + i * scan_msg.angle_increment)) / self.occupancy_map.info.resolution)
                grid_y= int((-self.grid_offset + range_measurement * np.sin( scan_msg.angle_min + i * scan_msg.angle_increment)) / self.occupancy_map.info.resolution)
                if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                    index = grid_x + grid_y * self.map_width
                    grid[index] = 100

        self.occupancy_map.data = grid

        return
    
    def dilate_occupancy_grid(self, dilation_radius):
        # Create a new grid to store the dilated occupancy grid
        dilated_grid = [0] * (self.map_height * self.map_width)

        # Iterate over each cell in the occupancy grid
        for grid_x in range(self.map_width):
            for grid_y in range(self.map_height):
                # If the cell is occupied
                if self.occupancy_map.data[grid_x + grid_y * self.map_width] == 100:
                    # Mark all cells within the dilation radius as occupied
                    for dx in range(-dilation_radius, dilation_radius + 1):
                        for dy in range(-dilation_radius, dilation_radius + 1):
                            if 0 <= grid_x + dx < self.map_width and 0 <= grid_y + dy < self.map_height:
                                dilated_grid[(grid_x + dx) + (grid_y + dy) * self.map_width] = 100

        # Replace the original occupancy grid with the dilated grid
        self.occupancy_map.data = dilated_grid


    def smooth_path(self, path, smoothing_factor=0.1):
        # Determine the degree of the spline
        k = min(3, len(path) - 1)
        path_np = path.T
        # Fit a B-spline to the path
        tck, u = splprep(path_np, u=None, s=smoothing_factor, k=k, per=0)
        u_new = np.linspace(u.min(), u.max(), len(path))
        x_new, y_new = splev(u_new, tck, der=0)
        smoothed_path = np.array(list(zip(x_new, y_new)))

        return smoothed_path

    def scan_callback(self, scan_msg):
        """
        LaserScan callback, you should update your occupancy grid here

        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:

        """
        # print("Received a scan callback")
        self.range_data = scan_msg
        self.get_oc_grid(scan_msg)
        self.dilate_occupancy_grid(dilation_radius = 1)
        self.grid_pub_.publish(self.occupancy_map)        



    def pose_callback(self, pose_msg):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.1

        # --------------------------------------------------------------------------------
        # Setting up start and end points
        self.current_pos = pose_msg.pose
        start = (pose_msg.pose.pose.position.x,pose_msg.pose.pose.position.y)

        target_idx = self.last_idx
        target_point = self.waypoints[target_idx,:]
        dx = target_point[0] - start[0]
        dy = target_point[1] - start[1]
        curr_dist = math.sqrt(dx * dx + dy * dy)

        while curr_dist < self.lookahead and target_idx < (self.waypoints.shape[0]) - 1:
            target_idx += 1
            target_point = self.waypoints[target_idx,:]
            dx = target_point[0] - start[0]
            dy = target_point[1] - start[1]
            curr_dist = math.sqrt(dx * dx + dy * dy)
        
        self.last_idx = target_idx
        
        end = target_point
        print(start, end)

        # Waypoint Node
        self.draw_marker(pose_msg.header.frame_id,pose_msg.header.stamp,end,self.target_point_pub, rgb = [1.0,0.0,0.0])
      
        # Compute the node
        # exit()
        tree = [0]
        tree[0] = node(start[0], start[1])
        tree[0].parent.append(start)

        isgoal = False
        i = 1
        start_time = time.time()
        while (isgoal is False): 
            sampled_point = self.sample(end)
            print('sample', sampled_point)
            # print('sample', rand)
            quaternion = np.array([
            pose_msg.pose.pose.orientation.x,
            pose_msg.pose.pose.orientation.y,
            pose_msg.pose.pose.orientation.z,
            pose_msg.pose.pose.orientation.w
            ])
            euler = self.euler_from_quaternion(quaternion)
            # rand_point = ((rand[1]*(-np.sin(euler[2])) + rand[0]*np.cos(euler[2])), (rand[1]*np.cos(euler[2]) + rand[0]*np.sin(euler[2])))
            # sampled_point = (rand_point[0] + start[0], rand_point[1] + start[1])
            # print('sample', sampled_point)

            nearest_ind = self.nearest(tree, sampled_point)
            nearest_node = tree[nearest_ind]
            new_node = self.steer(nearest_node, sampled_point)
            print('new:', new_node)
            # exit()

            # exit()
            iscollision = self.check_collision(nearest_node, new_node, euler[2], start)
            print(iscollision)
            # exit()
            if (iscollision is True):
                tree.append(i)
                tree[i] = node(new_node[0], new_node[1])
                tree[i].parent.append((nearest_ind, nearest_node.x, nearest_node.y))
                i += 1
                isgoal = self.is_goal(new_node, end[0], end[1])
            running_time = time.time()-start_time
            # if (running_time > 0.5):
            #     drive_msg.drive.speed = 0.1
            #     break
        # if running_time < 0.5:
        path = self.find_path(tree, tree[-1])
        path = np.array(path)
        # smoothed_path = self.smooth_path(path, smoothing_factor=0.3)
        # path = smoothed_path
        
        # Publish the Path
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for row in path:
            x, y = float(row[0]), float(row[1])
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)
    
        # Publish Tree
        self.rrt_publish_tree(tree)

        # Publish follow point
        target_idx = 0
        target_point = path[target_idx,:]
        dx = target_point[0] - start[0]
        dy = target_point[1] - start[1]
        curr_dist = math.sqrt(dx * dx + dy * dy)

        while curr_dist < 1 and target_idx < (path.shape[0]) - 1:
            target_idx += 1
            target_point = path[target_idx,:]
            dx = target_point[0] - start[0]
            dy = target_point[1] - start[1]
            curr_dist = math.sqrt(dx * dx + dy * dy)

        self.draw_marker(pose_msg.header.frame_id,pose_msg.header.stamp,target_point,self.follow_point_pub, rgb = [0.0,1.0,0.0])
      
        # Pure Pursuit
        delta_x = target_point[0] - pose_msg.pose.pose.position.x
        delta_y = target_point[1] - (pose_msg.pose.pose.position.y)
        desired_yaw = math.atan2(delta_y, delta_x)
        quaternion = np.array([
        pose_msg.pose.pose.orientation.x,
        pose_msg.pose.pose.orientation.y,
        pose_msg.pose.pose.orientation.z,
        pose_msg.pose.pose.orientation.w
        ])
        euler = self.euler_from_quaternion(quaternion)
        yaw_err = desired_yaw - euler[2]

        drive_msg.drive.steering_angle = yaw_err
        self.drive_pub.publish(drive_msg)

        # Opponent
        opp_drive_msg = AckermannDriveStamped()
        opp_drive_msg.drive.steering_angle = 0.0
        opp_drive_msg.drive.speed = 0.05
        self.opp_drive_pub.publish(opp_drive_msg)


    def sample(self, end):
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        
        x = np.random.uniform(end[0]-4.0, end[0]+3.0)
        y = np.random.uniform(end[1]-1.0, end[1]+1.0)

        # # Divide the grid into regions
        # num_regions = 10
        # region_width = self.occupancy_map.info.width // num_regions
        # region_height = self.occupancy_map.info.height // num_regions

        # # Create a list of free cells for each region
        # free_space_regions = [[] for _ in range(num_regions)]
        # for grid_x in range(self.occupancy_map.info.width):
        #     for grid_y in range(self.occupancy_map.info.height):
        #         if self.occupancy_map.data[grid_x + grid_y * self.occupancy_map.info.width] == 0:
        #             region_idx = ((grid_x // region_width) + (grid_y // region_height) * num_regions) % num_regions
        #             free_space_regions[region_idx].append((grid_x, grid_y))

        # # Randomly select a region that has free cells
        # free_regions = [i for i, region in enumerate(free_space_regions) if region]
        # if not free_regions:
        #     print("No Free Cells in Occupancy Grid?")
        #     return None  # Return None if there are no free cells
        # selected_region = random.choice(free_regions)

        # # Randomly select a cell within the selected region
        # rand_grid_x, rand_grid_y = random.choice(free_space_regions[selected_region])

        # return (rand_grid_x, rand_grid_y)
        
        return (x, y)


    def nearest(self, tree, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        dlist = [np.sqrt((tree[i].x - sampled_point[0])**2 + (tree[i].y - sampled_point[1])**2)
                 for i in range(len(tree))]

        minind = dlist.index(min(dlist))
        return minind

    def steer(self, nearest_node, sampled_point):   
        """
        This method should return a point in the viable set such that it is closer 
        to the nearest_node than sampled_point is.

        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """
        theta = np.arctan2(sampled_point[1] - nearest_node.y, sampled_point[0] - nearest_node.x)
        stepSize = 0.1
        x = nearest_node.x + stepSize*np.cos(theta)
        y = nearest_node.y + stepSize*np.sin(theta)
        new_node = (x,y)

        return new_node

    def check_collision(self, nearest_node, new_node, yaw, start):
        """
        This method should return whether the path between nearest and new_node is
        collision free.

        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                              with the occupancy grid
        """
        x1, y1 = nearest_node.x, nearest_node.y
        x1, y1 = start[0], start[1]
        x2, y2 = new_node[0], new_node[1]

        theta = np.arctan2(y2 - y1, x2 - x1)
    
        idx = ((theta-yaw)-self.range_data.angle_min)/self.range_data.angle_increment

        if int(idx)+40 > len(self.range_data.ranges)-1:
            return False
        else:
            for i in range(int(idx)-40,int(idx)+41):
                # print(self.range_data.ranges[i])
                if self.range_data.ranges[i] < np.sqrt((x2-x1)**2+(y2-y1)**2):
                    return False

        return True



    def is_goal(self, latest_added_node, goal_x, goal_y):
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enoughg to the goal
        """

        if np.sqrt((latest_added_node[0] - goal_x)**2 + (latest_added_node[1] - goal_y)**2) < 0.5:
            return True 
        else:
            return False

    def find_path(self, tree, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        path = []
        path.append([latest_added_node.x, latest_added_node.y])
        parent = latest_added_node.parent
        while parent[0][0] is not 0:
            path.append([parent[0][1], parent[0][2]])
            parent = tree[parent[0][0]].parent
        path.append([tree[0].x, tree[0].y])
        path = path[::-1]
        print(parent)
        return path

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


    # # The following methods are needed for RRT* and not RRT
    # def cost(self, tree, node):
    #     """
    #     This method should return the cost of a node

    #     Args:
    #         node (Node): the current node the cost is calculated for
    #     Returns:
    #         cost (float): the cost value of the node
    #     """
    #     return 0

    # def line_cost(self, n1, n2):
    #     """
    #     This method should return the cost of the straight line between n1 and n2

    #     Args:
    #         n1 (Node): node at one end of the straight line
    #         n2 (Node): node at the other end of the straint line
    #     Returns:
    #         cost (float): the cost value of the line
    #     """
    #     return 0

    # def near(self, tree, node):
    #     """
    #     This method should return the neighborhood of nodes around the given node

    #     Args:
    #         tree ([]): current tree as a list of Nodes
    #         node (Node): current node we're finding neighbors for
    #     Returns:
    #         neighborhood ([]): neighborhood of nodes as a list of Nodes
    #     """
    #     neighborhood = []
    #     return neighborhood


    def draw_marker(self, frame_id, stamp, position, publisher, rgb = [1.0,0.0,0.0], id=0):
        if position is None:
            return
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.id = id
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = 0.0
        publisher.publish(marker)


    def rrt_publish_tree(self, tree):
        for i,point in enumerate(tree):
            # Create a marker for the node
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = point.x#[0]
            marker.pose.position.y = point.y#[1]
            marker.scale.x = 0.05  # Smaller size for the nodes
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = 1.0  # Pink color for the nodes
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.id = i

            self.tree_pub.publish(marker)

            # If this node has a parent, create a marker for the path from this node to its parent
            if len(point.parent[0]) > 2 :

                path_marker = Marker()
                path_marker.header.frame_id = 'map'
                path_marker.type = Marker.LINE_STRIP
                path_marker.action = Marker.ADD
                path_marker.points.append(marker.pose.position)
                path_marker.points.append(Point(x=float(point.parent[0][1]), y=float(point.parent[0][2])))
                path_marker.scale.x = 0.02  # Thickness of the line
                path_marker.color.r = 0.0  # Blue color for the paths
                path_marker.color.g = 0.0
                path_marker.color.b = 1.0
                path_marker.color.a = 1.0
                path_marker.id = i + len(tree)  # Unique ID for each marker

                self.tree_pub.publish(path_marker)



def main(args=None):
    rclpy.init(args=args)
    print("RRT Initialized")
    rrt_node = RRT()
    rclpy.spin(rrt_node)

    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

