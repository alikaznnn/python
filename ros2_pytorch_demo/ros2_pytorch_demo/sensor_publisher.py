#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import math


class SensorPublisher(Node):
    """
    A ROS2 node that simulates sensor data publishing.
    Publishes 4 sensor values that could represent:
    - Distance sensors (ultrasonic, lidar, etc.)
    - Environmental data (temperature, humidity, etc.)
    - IMU data (acceleration, gyroscope, etc.)
    """

    def __init__(self):
        super().__init__('sensor_publisher')
        
        # Create publisher for sensor data
        self.publisher_ = self.create_publisher(
            Float32MultiArray, 
            'sensor_data', 
            10
        )
        
        # Timer to publish data at regular intervals
        timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Internal state for realistic sensor simulation
        self.counter = 0
        self.noise_level = 0.1
        
        self.get_logger().info('Sensor Publisher node has been started')

    def timer_callback(self):
        """
        Timer callback that publishes sensor data.
        Simulates realistic sensor readings with some patterns and noise.
        """
        msg = Float32MultiArray()
        
        # Simulate 4 different sensors with realistic patterns
        t = self.counter * 0.1  # Time in seconds
        
        # Sensor 1: Distance sensor with sinusoidal pattern (obstacles moving)
        distance = 2.0 + 1.0 * math.sin(0.5 * t) + np.random.normal(0, self.noise_level)
        
        # Sensor 2: Temperature sensor with slow drift
        temperature = 25.0 + 5.0 * math.sin(0.1 * t) + np.random.normal(0, self.noise_level)
        
        # Sensor 3: Light sensor with random fluctuations
        light = 500.0 + 100.0 * math.sin(0.3 * t) + np.random.normal(0, self.noise_level * 10)
        
        # Sensor 4: Battery voltage (slowly decreasing)
        battery = 12.0 - 0.001 * t + np.random.normal(0, self.noise_level * 0.1)
        
        # Package the data
        msg.data = [float(distance), float(temperature), float(light), float(battery)]
        
        # Publish the message
        self.publisher_.publish(msg)
        
        # Log every 50 messages (5 seconds at 10 Hz)
        if self.counter % 50 == 0:
            self.get_logger().info(
                f'Publishing sensor data: '
                f'Distance={distance:.2f}, Temp={temperature:.2f}, '
                f'Light={light:.2f}, Battery={battery:.2f}'
            )
        
        self.counter += 1


def main(args=None):
    """
    Main function to initialize and run the sensor publisher node.
    """
    rclpy.init(args=args)
    
    sensor_publisher = SensorPublisher()
    
    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()