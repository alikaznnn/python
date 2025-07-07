# ROS2 + PyTorch Demo Project

A simple demonstration project that combines **ROS2** (Robot Operating System 2), **PyTorch** (deep learning framework), and **Python** for AI-powered robotics applications.

## 🎯 Project Overview

This project demonstrates how to integrate AI/ML capabilities into robotics systems using:

- **ROS2**: For robot communication and coordination
- **PyTorch**: For neural network inference and training
- **Python**: As the primary programming language

### System Architecture

```
Sensor Data → AI Processing → Robot Control
     ↓              ↓              ↓
[SensorPublisher] → [AIProcessor] → [RobotController] → Robot/Simulator
```

## 🏗️ Project Structure

```
ros2_pytorch_demo/
├── package.xml                    # ROS2 package definition
├── setup.py                      # Python package setup
├── resource/                      # ROS2 resources
├── launch/
│   └── demo_launch.py            # Launch file to run all nodes
└── ros2_pytorch_demo/
    ├── __init__.py
    ├── neural_model.py            # PyTorch neural network definition
    ├── sensor_publisher.py        # Simulates sensor data
    ├── ai_processor.py           # AI inference using PyTorch
    └── robot_controller.py       # Robot safety and control
```

## 🚀 Features

### 1. **Sensor Publisher Node**
- Simulates realistic sensor data (distance, temperature, light, battery)
- Publishes data at 10 Hz with noise and realistic patterns
- Easy to extend with real sensor interfaces

### 2. **AI Processor Node**
- Uses PyTorch neural network for decision making
- Real-time inference on sensor data
- Publishes AI-generated robot commands
- Supports online learning capabilities

### 3. **Robot Controller Node**
- Safety monitoring and emergency stop functionality
- Command filtering and smoothing
- Configurable speed limits and safety timeouts
- Status monitoring and reporting

### 4. **PyTorch Integration**
- Simple feedforward neural network
- CPU/GPU automatic detection
- Easy to replace with more complex models
- Demonstrates real-time inference in robotics

## 🛠️ Prerequisites

### System Requirements
- Ubuntu 20.04+ or other ROS2-compatible OS
- Python 3.8+
- ROS2 (Foxy, Galactic, Humble, or newer)

### Python Dependencies
- torch >= 2.0.0
- torchvision >= 0.15.0
- numpy >= 1.21.0
- matplotlib >= 3.5.0

## 📦 Installation

### 1. Install ROS2
Follow the official ROS2 installation guide for your operating system:
```bash
# For Ubuntu (example for ROS2 Humble)
sudo apt update
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

### 2. Install Python Dependencies
```bash
pip install -r requirements.txt
```

### 3. Build the ROS2 Package
```bash
# Create a ROS2 workspace (if you don't have one)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone or copy this project
cp -r /path/to/ros2_pytorch_demo .

# Build the package
cd ~/ros2_ws
colcon build --packages-select ros2_pytorch_demo

# Source the workspace
source install/setup.bash
```

## 🎮 Usage

### Quick Start
```bash
# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Launch the complete system
ros2 launch ros2_pytorch_demo demo_launch.py
```

### Running Individual Nodes

#### 1. Start Sensor Publisher
```bash
ros2 run ros2_pytorch_demo sensor_publisher
```

#### 2. Start AI Processor
```bash
ros2 run ros2_pytorch_demo ai_processor
```

#### 3. Start Robot Controller
```bash
ros2 run ros2_pytorch_demo robot_controller
```

### Monitoring the System

#### View Active Topics
```bash
ros2 topic list
```

#### Monitor Sensor Data
```bash
ros2 topic echo /sensor_data
```

#### Monitor AI Commands
```bash
ros2 topic echo /ai_cmd_vel
```

#### Monitor Final Robot Commands
```bash
ros2 topic echo /cmd_vel
```

#### Monitor Robot Status
```bash
ros2 topic echo /robot_status
```

#### View Node Graph
```bash
rqt_graph
```

## 🔧 Configuration

### Modify Neural Network
Edit `ros2_pytorch_demo/neural_model.py` to:
- Change network architecture
- Load pre-trained weights
- Add new input/output dimensions
- Implement different learning algorithms

### Adjust Safety Parameters
Modify `robot_controller.py` to change:
- Maximum velocities
- Safety timeouts
- Filter parameters
- Emergency stop behavior

### Customize Sensor Data
Edit `sensor_publisher.py` to:
- Add new sensor types
- Change publishing frequency
- Modify noise characteristics
- Integrate real sensor drivers

## 📊 Example Output

```
[sensor_publisher]: Publishing sensor data: Distance=2.34, Temp=26.78, Light=567.23, Battery=11.95
[ai_processor]: AI Prediction #50: Linear=0.456, Angular=1.234 (from sensors: ['2.34', '26.78', '567.23', '11.95'])
[robot_controller]: Commands processed: 100 | Raw: (0.456, 1.234) | Final: (0.342, 0.987)
```

## 🤖 Real Robot Integration

To use with a real robot:

1. **Replace sensor_publisher** with real sensor drivers
2. **Connect robot_controller** to actual robot hardware
3. **Train the neural network** with real robot data
4. **Add sensor fusion** and state estimation
5. **Implement SLAM** for navigation capabilities

## 🔬 Extending the Project

### Add Computer Vision
```python
# Example: Add camera processing
import torchvision.transforms as transforms
from sensor_msgs.msg import Image

# Process camera images with PyTorch
def process_image(self, image_msg):
    # Convert ROS image to PyTorch tensor
    # Run inference with vision model
    # Publish detection results
```

### Add Reinforcement Learning
```python
# Example: Add RL agent
from stable_baselines3 import PPO

# Train agent with environment feedback
def update_policy(self, state, action, reward, next_state):
    # Update RL policy
    pass
```

### Add Multi-Robot Coordination
```python
# Example: Multi-robot communication
def coordinate_with_others(self, robot_states):
    # Implement swarm behavior
    # Share AI models between robots
    pass
```

## 🐛 Troubleshooting

### Common Issues

1. **PyTorch not found**
   ```bash
   pip install torch torchvision
   ```

2. **ROS2 package not found**
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

3. **Permission denied**
   ```bash
   chmod +x ros2_pytorch_demo/launch/demo_launch.py
   ```

4. **GPU not detected**
   - Install CUDA and cuDNN
   - Verify with: `python -c "import torch; print(torch.cuda.is_available())"`

## 📝 License

This project is licensed under the MIT License.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for:

- Bug fixes
- New features
- Documentation improvements
- Performance optimizations
- Additional examples

## 📚 References

- [ROS2 Documentation](https://docs.ros.org/)
- [PyTorch Documentation](https://pytorch.org/docs/)
- [ROS2 + AI/ML Best Practices](https://github.com/ros-ai)

---

**Happy coding! 🚀🤖**