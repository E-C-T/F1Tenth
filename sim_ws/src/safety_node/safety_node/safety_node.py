#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.
        # TODO: create ROS subscribers and publishers.

        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )
        self.odom_subscription

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.scan_subscription

        self.iTTC_threshold = 1.5

        #self.print_counter = 0
        #self.first_brake = False

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        ranges = scan_msg.ranges

        iTTC_array = self.calculate_iTTC(ranges, scan_msg)

        # Handle nan and inf values
        iTTC_array = [min(val, 100.0) if not np.isnan(val) and val != float('inf') else 100.0 for val in iTTC_array]

        # TODO: publish command to brake
        if any(iTTC < self.iTTC_threshold for iTTC in iTTC_array):
            print("BRAKE!!!!!!!!!!!!!")
            self.first_brake = True
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 0.0
            self.drive_publisher.publish(drive_msg)
        pass

    def calculate_iTTC(self, ranges, msg):
        iTTC_array = []

        for i, range_measurement in enumerate(ranges):
            angle = msg.angle_min + i * msg.angle_increment # in radians
            range_rate = -1* self.speed * np.cos(angle)

            # If range_rate is positive or zero => moving away, set to inf
            if range_rate < 0.0:
                iTTC = range_measurement / (-range_rate)
            else:
                iTTC = 100.0

            iTTC_array.append(iTTC)

            # if self.print_counter % 10 == 0 and self.first_brake == False:
            #         print(f'Angle:{angle:.3f}, iTTC: {iTTC:.3f},Range: {range_measurement:.3f}, Range_Rate: {range_rate:.3f}, V_x:{self.speed:.3f}')
 
        # self.print_counter+=1

        return iTTC_array


def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()