import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

import time
 
class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        odom_topic = '/ego_racecar/odom'

        # create subscribers and publishers
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        self.scan_subscription = self.create_subscription(
            LaserScan,
            lidarscan_topic,
            self.scan_callback,
            10
        )
        self.scan_subscription        

        self.odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )
        self.odom_subscription        

        # set PID gains
        self.kp = 6.0
        self.ki = 0.001
        self.kd = 0.5

        # store history
        self.integral = 0.0
        self.derivative = 0.0
        self.prev_error = 0.0
        self.error = 0.0

        self.speed = 0.0
        
        # 90 deg angle from x-axis
        self.b_angle = 90.0

        # Angle between a and b, 0 < theta < 70 degrees
        self.theta = 45.0 # should approximate to nearest radian angle of 0.783333532512188 in ranges

        self.b = 0.70
        self.a_1 = 1.25

        self.desired_dist = 0.75

        self.steering_angle = 0.0

        self.L = 0.65

        self.theta_old = 0.0

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x
        self.steering_angle = odom_msg.twist.twist.angular.z

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        # Convert the given angle to the closest index in the range_data array
        index = int((angle - range_data.angle_min) / range_data.angle_increment)

        # Check for NaN and infs
        if not np.isfinite(range_data.ranges[index]):
            print("range value is infinite!!!!!!")
            print("returning large finite value for correction")
            return 20.0

        return range_data.ranges[index]


    def get_error(self, range_data, dist):
        """
        Calculates the lookahead error, its derivative and integral to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated [error, integral_error, derivative_error]
        """
        self.prev_error = self.error
        
        # get distance to wall
        self.a_1 = self.get_range(range_data, np.deg2rad(self.b_angle-self.theta)) #range_data[720] for 45 degrees at approx 0.7853981633974483 radians
        self.b = self.get_range(range_data, np.deg2rad(self.b_angle)) #range_data[900] for 90 degrees at approx 1.5707963267948966 radians
        d_t = self.get_dist(self.a_1, self.b)

        self.error = dist - (d_t + self.L*np.sin(self.alpha))
        self.integral += self.error
        self.derivative = self.prev_error - self.error 
        
        print(f"self.error:{self.error:.4f}")
        print(f"self.integral error:{self.integral:.4f}")
        print(f"self.derivative error:{self.derivative:.4f}")

        return [self.error, self.integral, self.derivative]

    def get_dist(self, a, b):
        """
        Calculates the distance to the wall. 

        """
        
        self.alpha = np.arctan((a*np.cos(self.theta)-b)/(a*np.sin(self.theta)))
        d_t = b*np.cos(self.alpha)

        return d_t


    def pid_control(self, error):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error

        Returns:
            None, publishes Ackermann Drive msg with the new drive angle and velocity
        """

        V_theta = (self.kp*error[0]+self.ki*error[1]+self.kp*error[2])*0.5

        # calculate desired car velocity based on error
        theta_new = self.steering_angle - V_theta

        if np.abs(error[0]) < 0.1:
            theta_new = theta_new*0.1

        self.theta_old = theta_new

        if 0 <= np.abs(np.rad2deg(theta_new)) <= 10:
            velocity = 1.5  
        elif 10 < np.abs(np.rad2deg(theta_new)) <= 20:
            velocity = 1.0  
        else:
            velocity = 0.5 
            
        print(f"theta_new: {theta_new:.4f}, V_theta: {V_theta:.4f}")
        
        # fill in drive message and publish
        drive_msg = AckermannDriveStamped()

        drive_msg.drive.speed = velocity
        drive_msg.drive.steering_angle = theta_new
        self.drive_publisher.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """

        error = self.get_error(msg, self.desired_dist) 

        # actuate the car with PID
        self.pid_control(error) 


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()