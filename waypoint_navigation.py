#!/usr/bin/env python3

from swift_msgs.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy

class SwiftDroneController:
    def __init__(self):
        # Drone position and setpoint variables
        self.drone_position = [0.0, 0.0, 0.0]
        self.setpoint = [0, 0, 0]

        # Initialize drone commands
        self.cmd = swift_msgs()
        self.initialize_commands()

        # PID coefficients
        self.Kp = [109*0.06, 87*0.06, 1769*0.06]
        self.Ki = [0, 0, 24*0.008]
        self.Kd = [0, 0, 3515*0.3]

        # Error and PID variables
        self.error_x = 0.0
        self.error_y = 0.0
        self.error_z = 0.0
        # Previous errors for derivative term
        self.prev_error = [0, 0, 0]
        # Sum of errors for integral term
        self.sum_error = [0, 0, 0]
        
        # ROS publishers and subscribers
        self.command_pub = rospy.Publisher("/drone_command", swift_msgs, queue_size=1)
        self.alt_error_pub = rospy.Publisher("/alt_error", Float64, queue_size=1)
        self.roll_error_pub = rospy.Publisher("/roll_error", Float64, queue_size=1)
        self.pitch_error_pub = rospy.Publisher("/pitch_error", Float64, queue_size=1)
        rospy.Subscriber("whycon/poses", PoseArray, self.whycon_callback)
        rospy.Subscriber("/pid_tuning_altitude", PidTune, self.altitude_tuning_callback)
        rospy.Subscriber("/pid_tuning_pitch", PidTune, self.pitch_tuning_callback)
        rospy.Subscriber("/pid_tuning_roll", PidTune, self.roll_tuning_callback)

    def initialize_commands(self):
        # Initialize drone RC commands
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1500

    def whycon_callback(self, msg):
        # Whycon callback function to update drone position
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z

    def altitude_tuning_callback(self, data):
        # Callback function for altitude PID tuning parameters
        self.Kp[2] = data.Kp * 0.06
        self.Ki[2] = data.Ki * 0.0008
        self.Kd[2] = data.Kd * 0.3

    def pitch_tuning_callback(self, data):
        # Callback function for pitch PID tuning parameters
        self.Kp[0] = data.Kp * 0.06
        self.Ki[0] = data.Ki * 0.0008
        self.Kd[0] = data.Kd * 0.3

    def roll_tuning_callback(self, data):
        # Callback function for roll PID tuning parameters
        self.Kp[1] = data.Kp * 0.06
        self.Ki[1] = data.Ki * 0.0008
        self.Kd[1] = data.Kd * 0.3

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint

    def is_within_error_threshold(self, error_threshold):
        # Check if the drone is within the error range of the setpoint
        return abs(self.error_x) <= error_threshold and abs(self.error_y) <= error_threshold and abs(self.error_z) <= error_threshold

    def pid(self):
        # PID control logic
        self.error_x = -self.drone_position[0] + self.setpoint[0]
        self.error_y = self.drone_position[1] - self.setpoint[1]
        self.error_z = self.drone_position[2] - self.setpoint[2]

        # Proportional term
        pid_output_x = self.error_x * self.Kp[0]
        pid_output_y = self.error_y * self.Kp[1]
        pid_output_z = self.error_z * self.Kp[2]

        # Integral term
        self.sum_error[0] += self.error_x
        self.sum_error[1] += self.error_y
        self.sum_error[2] += self.error_z
        pid_output_z += self.sum_error[2] * self.Ki[2]

        # Derivative term
        pid_output_z += (self.error_z - self.prev_error[2]) * self.Kd[2]
        self.prev_error = [self.error_x, self.error_y, self.error_z]

        # Calculate new RC values
        self.cmd.rcRoll = int(1500 + pid_output_x)
        self.cmd.rcPitch = int(1500 + pid_output_y)
        self.cmd.rcThrottle = int(1500 + pid_output_z)

        # Limit throttle value between 1000 and 2000
        if self.cmd.rcThrottle > 2000:
            self.cmd.rcThrottle = 2000
        if self.cmd.rcThrottle < 1000:
            self.cmd.rcThrottle = 1000

        # Publish RC commands and error values
        self.command_pub.publish(self.cmd)
        self.alt_error_pub.publish(self.drone_position[2] - self.setpoint[2])
        self.roll_error_pub.publish(self.drone_position[1] - self.setpoint[1])
        self.pitch_error_pub.publish(self.drone_position[0] - self.setpoint[0])

class SetpointNavigator:
    def __init__(self):
        self.setpoints = [
            [0, 0, 23],
            [2, 0, 23],
            [2, 2, 23],
            [2, 2, 25],
            [-5, 2, 25],
            [-5, -3, 25],
            [-5, -3, 21],
            [7, -3, 21],
            [7, 0, 21],
            [0, 0, 19]
        ]
        self.current_setpoint_index = 0
        self.error_threshold = 0.2
        self.pid_controller = SwiftDroneController()

    def run(self):
        rate = rospy.Rate(30)  # Control loop frequency: 30 Hz
        while not rospy.is_shutdown():
            current_setpoint = self.setpoints[self.current_setpoint_index]
            self.pid_controller.set_setpoint(current_setpoint)

            # Check if the drone is within the error range of the current setpoint
            if self.pid_controller.is_within_error_threshold(self.error_threshold):
                # Move to the next setpoint
                self.current_setpoint_index += 1
                # Check if all setpoints have been reached
                if self.current_setpoint_index >= len(self.setpoints):
                    rospy.loginfo("All setpoints reached!")
                    break

            # Run the PID controller
            self.pid_controller.pid()
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("setpoint_navigator")  # Initialize node here
    navigator = SetpointNavigator()
    navigator.run()