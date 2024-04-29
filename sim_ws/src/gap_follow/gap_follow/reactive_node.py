import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
 
class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        self.drive_publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            lidarscan_topic,
            self.lidar_callback,
            10
        )
        self.lidar_subscription        


        self.proc_window_size = 5 # int [5,216,27,40]
        # Use a larger lookout and bubble radius for levine_blocked  8.0 and 1.2
        # Use a smaller lookout and bubble radius for levine_obs 1.05, 7.0
        self.bubble_radius = 1.05 # float
        self.lookout_d = 7.0 # float 

        #self.gap_param_d = 6.0 # float
        self.gap_param_n = 20 # int

        # self.theta_old = 0.0 # float

        # self.wall_buffer = 0.05 # float
        # self.theta_scale = 0.5

        # self.rqp_buffer = self.wall_buffer*0.5

        #self.counter = 0



    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        # Should high values be rejected first?
        proc_ranges = [r if r <= self.lookout_d else self.lookout_d for r in ranges]
        #proc_ranges = [r if r <= self.lookout_d else 0.0 for r in ranges]
        proc_ranges = [sum(ranges[i:i+self.proc_window_size]) / self.proc_window_size for i in range(0, len(ranges), self.proc_window_size)]
        proc_ranges = [avg_r for avg_r in proc_ranges for _ in range(self.proc_window_size)]

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        gap_idxs_list = []
        current_gap_idxs = []

        for i, value in enumerate(free_space_ranges):
            if value > self.bubble_radius:
                current_gap_idxs.append(i)
            else:
                if current_gap_idxs:
                    gap_idxs_list.append(current_gap_idxs)
                    current_gap_idxs = []

        # Check if the last sequence extends to the end
        if current_gap_idxs:
            gap_idxs_list.append(current_gap_idxs)

        # Filter out sequences shorter than gap_param_n
        gap_idxs_list_filtered = [indices for indices in gap_idxs_list if len(indices) >= self.gap_param_n]

        if not gap_idxs_list_filtered:
            print("NO SUITABLE GAPS!!!!! using Max nominal Gap")
            max_idx = np.argmax(np.array(free_space_ranges))
            max_gap_idxs = [int(max_idx-100), int(max_idx+100)] 

        else:
            # Find the gap with the highest average value
            max_gap_idxs = max(gap_idxs_list_filtered, key=lambda indices: sum(free_space_ranges[i] for i in indices) / len(indices))

        return [max_gap_idxs[0], max_gap_idxs[-1]]

    # def center_of_mass(self, indices, values):
    #     reduced_indices = indices[int(len(indices)*.1):int(len(indices)*.9)] 
    #     corrected_values = np.array(values)[reduced_indices]-self.bubble_radius
    #     #indices = np.array(indices)

    #     total_weight = sum(corrected_values)
    #     center_of_mass_indices = sum(idx * weight for idx, weight in zip(reduced_indices, corrected_values)) / total_weight 
    #     center_of_mass_ranges = sum(value * weight for value, weight in zip(values, corrected_values)) / total_weight
        
    #     return center_of_mass_indices, center_of_mass_ranges

    def center_of_mass(self, indices, values):
        #reduced_indices = indices[int(len(indices)*.1):int(len(indices)*.9)] 
        reduced_indices = indices
        corrected_values = np.array(values)[reduced_indices]-self.bubble_radius

        total_weight = sum(corrected_values)
        center_of_mass_index = sum(idx * weight for idx, weight in zip(reduced_indices, corrected_values)) / total_weight 
        center_of_mass_range = sum(value * weight for value, weight in zip(corrected_values, corrected_values)) / total_weight

        reduced_indices_com_idx = np.where(np.array(reduced_indices) == int(center_of_mass_index))[0]
        corrected_com_indices = reduced_indices[int(reduced_indices_com_idx-0.25*len(reduced_indices)):int(reduced_indices_com_idx+0.25*len(reduced_indices))]

        if len(corrected_com_indices) == 0:
            corrected_com_indices = np.arange((reduced_indices_com_idx -15),(reduced_indices_com_idx + 15))

        corrected_com_values = np.array(values)[corrected_com_indices]-self.bubble_radius
        corrected_total_weight = sum(corrected_com_values)

        corrected_com_index = sum(idx * weight for idx, weight in zip(corrected_com_indices,corrected_com_values)) / corrected_total_weight 
        corrected_com_range = sum(value * weight for value, weight in zip(corrected_com_values, corrected_com_values)) / corrected_total_weight

        return corrected_com_index, corrected_com_range
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """

        # gap_indices = np.arange(start_i,end_i+1)

        # gap_com_index, self.gap_com_range = self.center_of_mass(gap_indices, ranges)

        # # Turning Left
        # if int(gap_com_index) > 539:
        #     best_point = int(gap_com_index) *1.1
        # # Turning Right
        # elif int(gap_com_index) < 539:
        #     best_point = int(gap_com_index) *0.9
        
        # # Center
        # else:
        #     best_point = int(gap_com_index)

        # if self.gap_com_range < self.lookout_d:
        #     best_point = np.argmax(ranges[(corrected_com_index-20):(corrected_com_index+20))

        #Middle Point
        best_point = int((start_i+end_i)/2)
        return best_point

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges

        proc_ranges = self.preprocess_lidar(ranges)
        
        # TODO:
        #Find closest point to LiDAR 
        
        #Eliminate all points inside 'bubble' (set them to zero) 
        free_space_ranges =  [r if r >= self.bubble_radius else 0.0 for r in proc_ranges]

        #Find max length gap 
        #start_i, end_i = self.find_max_gap(free_space_ranges)
        max_gap = self.find_max_gap(free_space_ranges)

        if max_gap is None:
            theta_new = 0.0
        else:
            #Find the best point in the gap 
            #best_point = self.find_best_point(start_i, end_i, free_space_ranges)
            best_point = self.find_best_point(max_gap[0], max_gap[1], free_space_ranges)
            theta_new = data.angle_min + data.angle_increment*best_point # Radians
        #theta_new = theta_new*1.0


        # d_theta = abs(self.theta_old-theta_new)
 
       # VELOCITY TUNING 
       # Adjust speed based on gap com range, slow down for closer gaps
        # if 0.0 <= self.gap_com_range <= 1.0:
        #     velocity = 1.0
        # elif 1.0 < self.gap_com_range <= 2.0:
        #     velocity = 1.5
        # else:
        #     velocity = 2.0

        velocity = 1.0

        # Adjust speed based on change in steering angle, slow down when agressivly steering
        if 0 <= abs(np.rad2deg(theta_new)) <= 5:
            velocity = velocity*1.25 
            #theta_new = theta_new*0.5
        elif 5 < abs(np.rad2deg(theta_new)) <= 20:
            velocity = velocity*0.5 
            #theta_new = theta_new*0.75
        else:
            velocity = velocity*0.25
            #theta_new = theta_new*1.0

        # Slow down if obstacle directly in front of car
        if (np.array(proc_ranges[535:544]) < 1.0).any():
            #velocity = velocity*0.5
            velocity = 0.5
            #theta_new = theta_new*1.2
        

        # STEERING ANGLE TUNING
        # If obstacle is closer than half bubble radius from -45 to +45 deg, react
        # React to obstacle from center to 90 deg right
        # if (np.array(ranges[179:539]) < 0.5).any():
        #     #React to obstacle from center to 45 deg right
        #     if (np.array(ranges[359:539]) < 0.1).any(): 
        #         theta_new = theta_new + np.deg2rad(15)
        #         velocity = velocity*0.5
        #     else:
        #         theta_new = theta_new + np.deg2rad(15)

        # # React to obstacle from center to 90 deg left    
        # elif (np.array(ranges[540:900]) < 0.5).any():
        #     #React to obstacle from center to 45 deg left 
        #     if (np.array(ranges[540:720]) < 0.1).any():
        #         theta_new = theta_new - np.deg2rad(15)
        #         velocity = velocity*0.5
        #     else:
        #         theta_new = theta_new - np.deg2rad(15)


        # If average proc_range in the center 20 deg > 1m, (Implies chord window of > 0.347m)
        
        # forward_view = np.array(proc_ranges[499:580])
        # right_view = np.array(ranges[98:219])
        # left_view = np.array(ranges[860:981])
        # And the right side  (-90,-110) deg are within wall buffer slight adjustment
        # if np.mean(forward_view) > 1.0 and ((right_view < self.wall_buffer).any()):
        #     #theta_new = theta_new + np.deg2rad(15) #* self.theta_scale 
        #     anchors = [r if (r >= 1 and r <= 2) else 0.0 for r in forward_view]
        #     theta_new = data.angle_min + data.angle_increment*(np.argmax(anchors)+499)

        # # And the left side  (90,110) deg are within wall buffer slight adjustment
        # elif np.mean(forward_view) > 1.0 and ((left_view < self.wall_buffer).any()):
        #     #theta_new = theta_new - np.deg2rad(15) #* self.theta_scale 
        #     anchors = [r if (r >= 1 and r <= 2) else 0.0 for r in forward_view]
        #     theta_new = data.angle_min + data.angle_increment*(np.argmax(anchors)+499)

        # If there is an obstacle very close to rear quarter pannels >105 deg, < -105 deg
        # Keep moving forward
        # if (np.array(ranges[0:179]) < self.rqp_buffer).any() or (np.array(ranges[900:]) < self.rqp_buffer).any():
        #     theta_new = 0.0

        # if np.mean(forward_view) > 5.0 and (np.mean(left_view) - np.mean(right_view)) < 0.1:
        #     theta_new = 0.0
        #     velocity = 1.2


        # Stop if no suitable gap found
        if max_gap is None and theta_new == 0.0:
            velocity = 0.0
            

        # self.theta_old = theta_new

        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = velocity
        drive_msg.drive.steering_angle = theta_new
        self.drive_publisher.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
