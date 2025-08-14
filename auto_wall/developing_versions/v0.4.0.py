#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class SideWallFollower(object):
    def __init__(self):
        # track params
        self.track_width = 2.0
        self.corner_factor = 1.5

        # speed ctrl
        self.max_speed = 20.0
        self.min_speed = 3.0
        self.safe_distance = 3.0
        self.emergency_stop_dist = 0.3

        # PID ctrl
        self.Kp = 0.4
        self.Kd = 0.6
        self.Ki = 0.04
        self.prev_error = 0
        self.integral = 0

        # refinement ctrl
        self.speed_boost = 1.3
        self.brake_factor = 0.85
        self.corner_target = 0.2  # corner target dis
        self.straight_target = 1.0  # straight target dis

        # ROS init
        rospy.init_node("dual_side_wall_follower")
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
        rospy.loginfo("Dual Side Wall Follower Initialized!")

        # log setup
        self.log_dir = os.path.expanduser("~/.ros/racer_logs")
        os.makedirs(self.log_dir, exist_ok=True)
        self.log_file = os.path.join(self.log_dir,f"race_log_{time.strftime('%Y%m%d_%H%M%S')}.txt")
        self.setup_logging()

    def setup_logging(self):
        import logging
        self.logger = logging.getLogger('race_logger')
        self.logger.setLevel(logging.DEBUG)

        # file processor (records all levels)
        file_handler = logging.FileHandler(self.log_file)
        file_handler.setLevel(logging.DEBUG)
        file_format = logging.Formatter(
            '%(asctime)s [%(levelname)s] %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S')
        file_handler.setFormatter(file_format)

        # ROS processer (only INFO and above)
        ros_handler = logging.StreamHandler()
        ros_handler.setLevel(logging.INFO)

        self.logger.addHandler(file_handler)
        self.logger.addHandler(ros_handler)
        self.logger.info(f"Logging initialized. Writing to {self.log_file}")

    def log_sensor_data(self, sensor_dict):
        """package and record"""
        sensor_str = " | ".join([f"{k}:{v:.2f}m" for k,v in sensor_dict.items()])
        self.logger.debug(f"SENSORS: {sensor_str}")

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
        # dynamic balance factor (automatically adjusted based on deviation)
        dynamic_factor = np.clip(abs(right_deviation) / (abs(right_deviation) + abs(left_deviation) + 1e-5), 0.3, 0.7)
        return right_deviation * dynamic_factor - left_deviation * (1 - dynamic_factor)

    def calculate_wall_error(self, dis, a, track_condition, side):
        """calculate single wall deviation (turn:0.2m, straight:1.0m)"""
        alpha = np.arctan2((a * np.cos(np.pi/3) - dis), (a * np.sin(np.pi/3)))
        AB = dis * np.cos(alpha)
        D = AB + 3.0 * np.sin(alpha) # projection distance

        # dynamic target dis settings
        if track_condition == f"{side}_corner":
            target = 0.2
        else:
            target = 1.0

        # direction processing (left needs to be reversed)
        if side == "left":
            return (target - D) * -1
        else:
            return target - D

    def scan_callback(self, data):
        try:
            # obtain both sides range
            right_dis = self.get_range(data, -90)
            right_a = self.get_range(data, -30)
            left_dis = self.get_range(data, 90)
            left_a = self.get_range(data, 30)
            front_dis = self.get_range(data, 0)

            # package and record
            sensor_data = {
                'R90': right_dis,
                'R30': right_a,
                'L90': left_dis,
                'L30': left_a,
                'Front': front_dis
            }
            self.log_sensor_data(sensor_data)

            # calculate both sides deviation
            right_deviation = self.calculate_wall_error(right_dis, right_a, track_condition, "right")
            left_deviation = self.calculate_wall_error(left_dis, left_a, track_condition, "left")

            if track_condition == "right_corner":
                balanced_error = right_deviation * 0.9 # mainly refer to the right
            elif track_condition == "left_corner":
                balanced_error = -left_deviation * 0.9 # mainly refer to the left (take the negative sign to unify the direction)
            else:
                balanced_error = self.balance_control(right_deviation, left_deviation) # dynamic factor

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
                drive_msg.drive.speed = np.clip(speed, 8, 20)
                self.drive_pub.publish(drive_msg)
            elif track_condition == "right_corner": # right
                base_speed *= self.brake_factor
                drive_msg.drive.steering_angle = np.clip(steering_angle * min(1.0, self.track_width / 5), -0.5, 0.2)
                drive_msg.drive.speed = np.clip(speed, 3, 8)
                self.drive_pub.publish(drive_msg)
            else: # left
                base_speed *= self.brake_factor
                drive_msg.drive.steering_angle = np.clip(steering_angle * min(1.0, self.track_width / 5), -0.2, 0.5)
                drive_msg.drive.speed = np.clip(speed, 3, 8)
                self.drive_pub.publish(drive_msg)

            # debug info
            self.logger.info(
                f"CONTROL | Cond:{track_condition:8s} | "
                f"Target:{'0.2m' if track_condition != 'straight' else '1.0m'} | "
                f"Speed:{speed:.2f}m/s | "
                f"Steer:{np.degrees(steering_angle):.1f}°")

            if abs(balanced_error) > 1.0:
                self.logger.warning(
                    f"Large error! R:{right_deviation:.2f} L:{left_deviation:.2f}")

        except Exception as e:
            self.logger.error(f"Control error: {str(e)}", exc_info=True)
            rospy.logerr(f"Control error: {str(e)}") # original ROS error output

if __name__ == '__main__':
    try:
        follower = SideWallFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated")
