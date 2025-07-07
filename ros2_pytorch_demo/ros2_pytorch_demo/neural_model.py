#!/usr/bin/env python3

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np


class SimpleRobotNet(nn.Module):
    """
    A simple neural network for robot decision making.
    Takes sensor readings and predicts robot actions.
    """
    
    def __init__(self, input_size=4, hidden_size=16, output_size=2):
        super(SimpleRobotNet, self).__init__()
        self.fc1 = nn.Linear(input_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.fc3 = nn.Linear(hidden_size, output_size)
        self.dropout = nn.Dropout(0.1)
        
    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = self.dropout(x)
        x = F.relu(self.fc2(x))
        x = self.dropout(x)
        x = torch.tanh(self.fc3(x))  # Output between -1 and 1
        return x


class RobotAI:
    """
    AI processor that wraps the neural network and provides
    easy-to-use methods for ROS2 integration.
    """
    
    def __init__(self):
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = SimpleRobotNet().to(self.device)
        self.model.eval()  # Set to evaluation mode
        
        # Initialize with some trained weights (in a real scenario, you'd load these)
        self._initialize_pretrained_weights()
        
    def _initialize_pretrained_weights(self):
        """
        Initialize the model with some reasonable weights.
        In a real application, you would load pre-trained weights here.
        """
        with torch.no_grad():
            # Simple initialization that creates reasonable behavior
            self.model.fc1.weight.fill_(0.1)
            self.model.fc2.weight.fill_(0.1)
            self.model.fc3.weight.fill_(0.5)
    
    def predict_action(self, sensor_data):
        """
        Predict robot action based on sensor data.
        
        Args:
            sensor_data: List or array of 4 sensor values
            
        Returns:
            Tuple of (linear_velocity, angular_velocity)
        """
        # Convert to tensor and add batch dimension
        input_tensor = torch.FloatTensor(sensor_data).unsqueeze(0).to(self.device)
        
        with torch.no_grad():
            output = self.model(input_tensor)
            linear_vel = float(output[0, 0]) * 2.0  # Scale to reasonable velocity
            angular_vel = float(output[0, 1]) * 3.14  # Scale to reasonable angular velocity
            
        return linear_vel, angular_vel
    
    def update_model(self, sensor_data, target_action):
        """
        Update the model with new training data (online learning).
        This is a simplified version - in practice you'd use proper training loops.
        """
        self.model.train()
        
        input_tensor = torch.FloatTensor(sensor_data).unsqueeze(0).to(self.device)
        target_tensor = torch.FloatTensor(target_action).unsqueeze(0).to(self.device)
        
        optimizer = torch.optim.Adam(self.model.parameters(), lr=0.001)
        criterion = nn.MSELoss()
        
        # Simple one-step update
        optimizer.zero_grad()
        output = self.model(input_tensor)
        loss = criterion(output, target_tensor)
        loss.backward()
        optimizer.step()
        
        self.model.eval()
        return float(loss.item())