#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv

 
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
        # create ROS subscribers and publishers
        drive_topic = '/drive'
        odom_topic = '/ego_racecar/odom'

        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped, 
            drive_topic, 
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.pose_callback,
            10
        )
        self.odom_subscription      

        self.waypoints = self.load_waypoints('/sim_ws/src/f1tenth_gym_ros/rcws/logs/wp-2023-10-14-18-11-08.csv')
        self.last_waypoint_index = 0

        self.lookahead = 0.5
        self.K = 2.0 # proportional gain for speed

    def load_waypoints(self, file_path):
        waypoints = []
        with open(file_path, 'r') as f:
            csv_reader = csv.reader(f, delimiter=',')
            for row in csv_reader:
                waypoint = PoseStamped()
                waypoint.pose.position.x = float(row[0])
                waypoint.pose.position.y = float(row[1])
                waypoints.append(waypoint)
        return waypoints

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

    def pose_callback(self, pose_msg):
        # find the current waypoint to track 
        quaternion = np.array([
            pose_msg.pose.pose.orientation.x,
            pose_msg.pose.pose.orientation.y,
            pose_msg.pose.pose.orientation.z,
            pose_msg.pose.pose.orientation.w])

        _,_,yaw = self.euler_from_quaternion(quaternion)

        lookahead_x = pose_msg.pose.pose.position.x + self.lookahead * np.cos(yaw)
        lookahead_y = pose_msg.pose.pose.position.y + self.lookahead * np.sin(yaw)

        for waypoint in self.waypoints[self.last_waypoint_index:]:
            distance = np.hypot(waypoint.pose.position.x - lookahead_x, waypoint.pose.position.y - lookahead_y)
            if distance > self.lookahead:
                break
            self.last_waypoint_index += 1

        if self.last_waypoint_index+40 >= len(self.waypoints):
            self.last_waypoint_index = 300 # hacky fix for these particular waypoints, skips pre loop waypoints
            for waypoint in self.waypoints[self.last_waypoint_index:]:
                    distance = np.hypot(waypoint.pose.position.x - lookahead_x, waypoint.pose.position.y - lookahead_y)
                    if distance > self.lookahead:
                        break
                    self.last_waypoint_index += 1            


        # transform goal point to vehicle frame of reference
        dx = waypoint.pose.position.x - pose_msg.pose.pose.position.x
        dy = waypoint.pose.position.y - pose_msg.pose.pose.position.y

        goal_x = dx * np.cos(yaw) + dy * np.sin(yaw)
        goal_y = dy * np.cos(yaw) - dx * np.sin(yaw)


        # calculate curvature/steering angle
        dl = np.hypot(goal_x, goal_y)
        steering_angle = 2 * goal_y / dl**2

        # publish drive message, don't forget to limit the steering angle.
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle*0.7
        drive_msg.drive.speed = 1.0 * self.K**2/dl**2 

        # Limit the steering angle
        max_steering_angle = np.pi/6 # 30 deg
        if abs(drive_msg.drive.steering_angle) > max_steering_angle:
            drive_msg.drive.steering_angle = max_steering_angle * np.sign(drive_msg.drive.steering_angle)

        self.drive_publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
