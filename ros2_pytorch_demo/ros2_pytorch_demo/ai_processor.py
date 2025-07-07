#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from .neural_model import RobotAI
import numpy as np


class AIProcessor(Node):
    """
    A ROS2 node that uses PyTorch to process sensor data and generate robot commands.
    This demonstrates the integration of AI/ML with ROS2 for autonomous robotics.
    """

    def __init__(self):
        super().__init__('ai_processor')
        
        # Initialize the PyTorch AI model
        try:
            self.robot_ai = RobotAI()
            self.get_logger().info('PyTorch model initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize PyTorch model: {e}')
            return
        
        # Subscriber for sensor data
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'sensor_data',
            self.sensor_callback,
            10
        )
        
        # Publisher for AI-generated robot commands
        self.cmd_publisher = self.create_publisher(
            Twist,
            'ai_cmd_vel',
            10
        )
        
        # Publisher for processed sensor data (for debugging/visualization)
        self.processed_data_publisher = self.create_publisher(
            Float32MultiArray,
            'processed_sensor_data',
            10
        )
        
        # Statistics tracking
        self.message_count = 0
        self.last_prediction = [0.0, 0.0]
        
        self.get_logger().info('AI Processor node has been started')

    def sensor_callback(self, msg):
        """
        Callback function that processes incoming sensor data using PyTorch.
        """
        try:
            # Extract sensor data
            sensor_data = list(msg.data)
            
            if len(sensor_data) != 4:
                self.get_logger().warn(f'Expected 4 sensor values, got {len(sensor_data)}')
                return
            
            # Normalize sensor data for better neural network performance
            normalized_data = self.normalize_sensor_data(sensor_data)
            
            # Use PyTorch model to predict robot action
            linear_vel, angular_vel = self.robot_ai.predict_action(normalized_data)
            
            # Create and publish robot command
            cmd_msg = Twist()
            cmd_msg.linear.x = linear_vel
            cmd_msg.angular.z = angular_vel
            self.cmd_publisher.publish(cmd_msg)
            
            # Publish processed data for visualization
            processed_msg = Float32MultiArray()
            processed_msg.data = normalized_data + [linear_vel, angular_vel]
            self.processed_data_publisher.publish(processed_msg)
            
            # Store last prediction
            self.last_prediction = [linear_vel, angular_vel]
            self.message_count += 1
            
            # Log periodically
            if self.message_count % 50 == 0:
                self.get_logger().info(
                    f'AI Prediction #{self.message_count}: '
                    f'Linear={linear_vel:.3f}, Angular={angular_vel:.3f} '
                    f'(from sensors: {[f"{x:.2f}" for x in sensor_data]})'
                )
                
        except Exception as e:
            self.get_logger().error(f'Error in AI processing: {e}')

    def normalize_sensor_data(self, sensor_data):
        """
        Normalize sensor data to improve neural network performance.
        
        Args:
            sensor_data: List of 4 sensor values [distance, temperature, light, battery]
            
        Returns:
            List of normalized sensor values
        """
        # Define typical ranges for normalization
        ranges = {
            0: (0.0, 5.0),    # Distance sensor (0-5 meters)
            1: (15.0, 35.0),  # Temperature (15-35 Celsius)
            2: (0.0, 1000.0), # Light sensor (0-1000 lux)
            3: (10.0, 13.0)   # Battery voltage (10-13 volts)
        }
        
        normalized = []
        for i, value in enumerate(sensor_data):
            min_val, max_val = ranges[i]
            # Normalize to [-1, 1] range
            normalized_val = 2.0 * (value - min_val) / (max_val - min_val) - 1.0
            # Clamp to range
            normalized_val = max(-1.0, min(1.0, normalized_val))
            normalized.append(normalized_val)
        
        return normalized

    def get_model_info(self):
        """
        Get information about the current PyTorch model.
        """
        if hasattr(self, 'robot_ai'):
            param_count = sum(p.numel() for p in self.robot_ai.model.parameters())
            device = str(self.robot_ai.device)
            return f"Model parameters: {param_count}, Device: {device}"
        return "Model not initialized"


def main(args=None):
    """
    Main function to initialize and run the AI processor node.
    """
    rclpy.init(args=args)
    
    ai_processor = AIProcessor()
    
    # Print model information
    if hasattr(ai_processor, 'robot_ai'):
        ai_processor.get_logger().info(ai_processor.get_model_info())
    
    try:
        rclpy.spin(ai_processor)
    except KeyboardInterrupt:
        pass
    finally:
        ai_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()