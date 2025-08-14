#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class SideWallFollower(Object):
    def __init__(self):
        # track params
        self.track_width = 2.0
        self.corner_factor = 1.5

        # speed ctrl
        self.max_speed = 15.0
        self.min_speed = 3.0
        self.safe_distance = 3.0
        self.emergency_stop_dist = 0.6

        # PID ctrl
        self.Kp = 0.4
        self.Kd = 0.6
        self.Ki = 0.04
        self.prev_error = 0
        self.integral = 0

        # refinement ctrl
        self.speed_boost = 1.3
        self.brake_factor = 0.85
        self.corner_target = 0.2  # corner target range
        self.straight_target = 1.0  # straight target range

        # ROS init
        rospy.init_node("dual_side_wall_follower")
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
        rospy.loginfo("Dual Side Wall Follower Initialized!")

    def get_range(self, data, angle_deg):
        """obtain effective radar range"""
        angle_rad = np.deg2rad(angle_deg)
        idx = int((angle_rad - data.angle_min) / data.angle_increment)
        dis = data.ranges[idx]
        if (data.range_min <= dis <= data.range_max):
            return dis
        else:
            return 10.0

    def detect_track_condition(self, right_dis, left_dis, front_dis):
        if front_dis < self.emergency_stop_dist:
            return "emergency"
        # both sides
        right_open = right_dis > (self.track_width/2) * self.corner_factor
        left_open = left_dis > (self.track_width/2) * self.corner_factor

        if right_open and not left_open:
            return "right_corner"
        elif left_open and not right_open:
            return "left_corner"
        else:
            return "straight"

    def balance_control(self, right_deviation, left_deviation):
        """bilateral deviation fusion"""
        # Dynamic balance factor (automatically adjusted based on deviation)
        dynamic_factor = np.clip(abs(right_deviation) / (abs(right_deviation) + abs(left_deviation) + 1e-5), 0.3, 0.7)
        return right_deviation * dynamic_factor - left_deviation * (1 - dynamic_factor)

    def calculate_wall_error(self, dis, a):
        """calculate single wall deviation"""
        alpha = np.arctan((a * np.cos(np.pi/3) - dis) / (a * np.sin(np.pi/3)))
        AB = dis * np.cos(alpha)
        track_condition = self.detect_track_condition(right_dis, left_dis, front_dis)
        error_factor = (AB + 3.0 * np.sin(alpha)) # deviation = target(1m) - projection distance
        if track_condition == "right_corner": # right
            return 0.1 - error_factor
        elif track_condition == "left_corner": # left
            return -(0.1 - error_factor)
        else: # straight
            return 1.0 - error_factor

    def scan_callback(self, data):
        try:
            # obtain both sides range
            right_dis = self.get_range(data, -90)
            right_a = self.get_range(data, -30)
            left_dis = self.get_range(data, 90) # the value at thirty degrees
            left_a = self.get_range(data, 30) # the value at thirty degrees
            front_dis = self.get_range(data, 0)

            # calculate both sides deviation
            right_deviation = self.calculate_wall_error(right_dis, right_a)
            left_deviation = self.calculate_wall_error(left_dis, left_a)

            balanced_error = self.balance_control(right_deviation, left_deviation) # mix both sides deviation

            track_condition = self.detect_track_condition(right_dis, left_dis, front_dis) # detect track condition

            # PID ctrl
            self.integral = np.clip(self.integral + balanced_error, -0.5, 0.5)
            derivative = balanced_error - self.prev_error
            steering_angle = (self.Kp * balanced_error + self.Ki * self.integral + self.Kd * derivative)
            self.prev_error = balanced_error

            # dynamic speed ctrl and calculate
            drive_msg = AckermannDriveStamped()
            speed_coe = 3.0 # dis-speed double factor
            base_speed = self.min_speed + (front_dis / self.safe_distance) * speed_coe
            speed = base_speed * (1.0 - (abs(steering_angle) / 0.5) * self.brake_factor)
            speed = np.clip(speed, self.min_speed, self.max_speed)

            # dynamic ctrl
            if track_condition == "straight": # straight
                base_speed *= self.speed_boost
                drive_msg.drive.steering_angle = np.clip(steering_angle * min(1.0, self.track_width / 5), -0.1, 0.1)
                drive_msg.drive.speed = speed
                self.drive_pub.publish(drive_msg)
            elif track_condition == "right_corner": # right
                base_speed *= self.brake_factor
                drive_msg.drive.steering_angle = np.clip(steering_angle * min(1.0, self.track_width / 5), -0.5, 0.2)
                drive_msg.drive.speed = speed
                self.drive_pub.publish(drive_msg)
            else: # left
                base_speed *= self.brake_factor
                drive_msg.drive.steering_angle = np.clip(steering_angle * min(1.0, self.track_width / 5), -0.2, 0.5)
                drive_msg.drive.speed = speed
                self.drive_pub.publish(drive_msg)

            # debug info
            rospy.loginfo_throttle(0.5,
                f"State: {track_condition:8s} | "
                f"Speed: {speed:.2f}m/s | "
                f"Steer: {np.degrees(steering_angle):.1f}° | "
                f"Errors: R{right_deviation:.2f}/L{left_deviation:.2f}→{balanced_error:.2f}")

        except Exception as e:
            rospy.logerr(f"Control error: {str(e)}")

if __name__ == '__main__':
    try:
        follower = SideWallFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated")
