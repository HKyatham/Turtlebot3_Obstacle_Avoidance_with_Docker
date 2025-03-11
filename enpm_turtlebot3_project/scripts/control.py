#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pid_controller import PIDController


def main(args=None):
    rclpy.init(args=args)
    start_index = (100, 100)
    goal_index = (650, 170)
    Step = 5
    clerance = int(input("Enter the clerance distance in cm."))
    Robot_radius = 20
    # Pass the path to the PID controller
    pid_controller = PIDController(start_index, goal_index, Step, clerance=clerance + Robot_radius)
    try:
        
        rclpy.spin(pid_controller)
    except:
        pid_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
