#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String
import math


class RobotController(Node):
    """
    A ROS2 node that acts as a robot controller.
    It receives AI-generated commands and manages the robot's behavior,
    including safety checks and command filtering.
    """

    def __init__(self):
        super().__init__('robot_controller')
        
        # Subscriber for AI-generated commands
        self.ai_cmd_subscription = self.create_subscription(
            Twist,
            'ai_cmd_vel',
            self.ai_command_callback,
            10
        )
        
        # Publisher for final robot commands (to actual robot or simulator)
        self.cmd_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # Publisher for robot status
        self.status_publisher = self.create_publisher(
            String,
            'robot_status',
            10
        )
        
        # Safety and control parameters
        self.max_linear_speed = 2.0   # m/s
        self.max_angular_speed = 3.14  # rad/s
        self.emergency_stop = False
        self.safety_timeout = 2.0  # seconds
        
        # Command filtering (simple low-pass filter)
        self.filter_alpha = 0.3
        self.filtered_linear = 0.0
        self.filtered_angular = 0.0
        
        # Statistics
        self.commands_received = 0
        self.commands_sent = 0
        self.last_command_time = self.get_clock().now()
        
        # Timer for safety monitoring and status updates
        self.safety_timer = self.create_timer(0.1, self.safety_check)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Robot Controller node has been started')

    def ai_command_callback(self, msg):
        """
        Callback for AI-generated commands.
        Applies safety checks and filtering before sending to robot.
        """
        self.commands_received += 1
        self.last_command_time = self.get_clock().now()
        
        try:
            # Extract command values
            raw_linear = msg.linear.x
            raw_angular = msg.angular.z
            
            # Apply safety limits
            safe_linear = self.apply_safety_limits(raw_linear, self.max_linear_speed)
            safe_angular = self.apply_safety_limits(raw_angular, self.max_angular_speed)
            
            # Apply command filtering (smooth out rapid changes)
            self.filtered_linear = self.apply_filter(self.filtered_linear, safe_linear)
            self.filtered_angular = self.apply_filter(self.filtered_angular, safe_angular)
            
            # Create final command
            final_cmd = Twist()
            
            if not self.emergency_stop:
                final_cmd.linear.x = self.filtered_linear
                final_cmd.angular.z = self.filtered_angular
            else:
                # Emergency stop - all velocities to zero
                final_cmd.linear.x = 0.0
                final_cmd.angular.z = 0.0
            
            # Publish the final command
            self.cmd_publisher.publish(final_cmd)
            self.commands_sent += 1
            
            # Log periodically
            if self.commands_received % 100 == 0:
                self.get_logger().info(
                    f'Commands processed: {self.commands_received} | '
                    f'Raw: ({raw_linear:.3f}, {raw_angular:.3f}) | '
                    f'Final: ({final_cmd.linear.x:.3f}, {final_cmd.angular.z:.3f})'
                )
                
        except Exception as e:
            self.get_logger().error(f'Error processing AI command: {e}')

    def apply_safety_limits(self, value, max_value):
        """
        Apply safety limits to command values.
        """
        return max(-max_value, min(max_value, value))

    def apply_filter(self, current_filtered, new_value):
        """
        Apply a simple low-pass filter to smooth commands.
        """
        return self.filter_alpha * new_value + (1 - self.filter_alpha) * current_filtered

    def safety_check(self):
        """
        Perform safety checks and monitoring.
        """
        current_time = self.get_clock().now()
        time_since_last_cmd = (current_time - self.last_command_time).nanoseconds / 1e9
        
        # Check for command timeout
        if time_since_last_cmd > self.safety_timeout:
            if not self.emergency_stop:
                self.get_logger().warn(f'No AI commands received for {time_since_last_cmd:.1f}s - Emergency stop activated')
                self.emergency_stop = True
                
                # Send stop command
                stop_cmd = Twist()
                stop_cmd.linear.x = 0.0
                stop_cmd.angular.z = 0.0
                self.cmd_publisher.publish(stop_cmd)
        else:
            if self.emergency_stop:
                self.get_logger().info('AI commands resumed - Emergency stop deactivated')
                self.emergency_stop = False

    def publish_status(self):
        """
        Publish robot status information.
        """
        status_msg = String()
        
        uptime = self.get_clock().now().nanoseconds / 1e9
        success_rate = (self.commands_sent / max(1, self.commands_received)) * 100
        
        status = {
            'emergency_stop': self.emergency_stop,
            'commands_received': self.commands_received,
            'commands_sent': self.commands_sent,
            'success_rate': f'{success_rate:.1f}%',
            'uptime': f'{uptime:.1f}s',
            'current_speed': f'({self.filtered_linear:.3f}, {self.filtered_angular:.3f})'
        }
        
        status_msg.data = str(status)
        self.status_publisher.publish(status_msg)

    def set_emergency_stop(self, stop=True):
        """
        Manually set emergency stop state.
        """
        self.emergency_stop = stop
        if stop:
            self.get_logger().warn('Manual emergency stop activated')
        else:
            self.get_logger().info('Manual emergency stop deactivated')

    def update_safety_params(self, max_linear=None, max_angular=None, timeout=None):
        """
        Update safety parameters.
        """
        if max_linear is not None:
            self.max_linear_speed = max_linear
            self.get_logger().info(f'Max linear speed updated to {max_linear}')
        
        if max_angular is not None:
            self.max_angular_speed = max_angular
            self.get_logger().info(f'Max angular speed updated to {max_angular}')
            
        if timeout is not None:
            self.safety_timeout = timeout
            self.get_logger().info(f'Safety timeout updated to {timeout}s')


def main(args=None):
    """
    Main function to initialize and run the robot controller node.
    """
    rclpy.init(args=args)
    
    robot_controller = RobotController()
    
    # Log initial configuration
    robot_controller.get_logger().info(
        f'Robot Controller initialized with: '
        f'Max linear speed: {robot_controller.max_linear_speed} m/s, '
        f'Max angular speed: {robot_controller.max_angular_speed} rad/s, '
        f'Safety timeout: {robot_controller.safety_timeout}s'
    )
    
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()