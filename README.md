# 🚀 ROS2 Docker Container with GPU Acceleration

> **A complete ROS2 Humble development environment with NVIDIA GPU support, GUI applications, and hardware integration**

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue) ![Docker](https://img.shields.io/badge/Docker-Ready-green) ![NVIDIA](https://img.shields.io/badge/NVIDIA-GPU%20Accelerated-brightgreen) ![Status](https://img.shields.io/badge/Status-Ready%20to%20Use-success)

## 🎯 Quick Start (TL;DR)

```bash
# 1. Build the container
sudo docker build -t shriman1:a .

# 2. Run with GPU acceleration
sudo ./run2.bash

# 3. Inside container - test everything works
source /opt/ros/humble/setup.bash
nvidia-smi  # Check GPU
rviz2       # Launch visualization
```

---

## 📋 What You Get

| Feature | Status | Description |
|---------|--------|-------------|
| 🤖 **ROS2 Humble** | ✅ Ready | Full desktop installation with all tools |
| 🎮 **NVIDIA GPU** | ✅ RTX 4070 Ti | Hardware-accelerated simulations |
| 🖥️ **GUI Support** | ✅ X11 | RViz2, Gazebo, and all graphical apps |
| 🔧 **Hardware Access** | ✅ USB | Connect real robots and sensors |
| 🏗️ **Gazebo Sim** | ✅ GPU Accelerated | Fast physics and realistic rendering |
| 📡 **Arduino CLI** | ✅ Included | Microcontroller development tools |

---

## 🛠️ System Requirements

### ✅ **Your System Status**
- **OS**: Linux (Ubuntu/Debian) ✅
- **GPU**: NVIDIA GeForce RTX 4070 Ti ✅ 
- **Docker**: Installed ✅
- **NVIDIA Runtime**: Configured ✅
- **X11**: Available ✅

### 🔧 **If Setting Up From Scratch**
<details>
<summary>Click to expand setup instructions</summary>

```bash
# Install Docker (if needed)
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Install NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```
</details>

---

## 🚀 Getting Started

### 📦 **Step 1: Build the Container**
```bash
cd /path/to/this/directory
sudo docker build -t shriman1:a .
```
*Building takes ~5-10 minutes depending on your internet speed*

### 🎮 **Step 2: Choose Your Launch Method**

#### **Option A: Smart Auto-Detection (Recommended)**
```bash
sudo ./run2.bash
```
✨ *Automatically detects if NVIDIA GPU is available and uses it*

#### **Option B: Force No-GPU Mode**
```bash
sudo ./run_no_nvidia.bash
```
🔄 *Use this if you want to run without GPU acceleration*

### 🎯 **Step 3: Verify Everything Works**

Once inside the container:
```bash
# Check ROS2 installation
source /opt/ros/humble/setup.bash
ros2 --help

# Verify GPU access (should show RTX 4070 Ti)
nvidia-smi

# Test GUI applications
rviz2 &          # 3D visualization tool
gazebo &         # Physics simulation
rqt &            # ROS GUI tools

# Check Arduino CLI
arduino-cli version
```

---

## 📁 Project Structure

```
ros2-docker/
├── 🐳 Dockerfile              # Container definition
├── 📄 README.md               # This documentation
├── 🚀 run2.bash               # Smart launcher (auto-detects GPU)
├── 🔧 run_no_nvidia.bash      # CPU-only launcher
├── 📜 new_run.bash            # Legacy script (reference)
└── 📂 bin/
    └── 🔌 arduino-cli         # Microcontroller tools
```

---

## 🎮 What You Can Do

### 🤖 **Robot Development**
- **Simulate robots** in Gazebo with realistic physics
- **Visualize sensor data** in RViz2 with smooth 3D rendering
- **Connect real hardware** via USB (sensors, microcontrollers)
- **Develop ROS2 nodes** with full IDE support

### 🏗️ **AI/ML Robotics**
- **CUDA acceleration** for neural networks
- **GPU-accelerated computer vision** with OpenCV
- **Real-time processing** for autonomous navigation
- **Machine learning** model training and inference

### 🔧 **Hardware Integration**
- **Arduino programming** with included CLI tools
- **Sensor integration** through USB passthrough
- **Real robot control** with direct hardware access
- **Custom electronics** development and testing

---

## 🛟 Troubleshooting

### ❓ **Common Issues & Solutions**

<details>
<summary><strong>🖥️ GUI Applications Won't Start</strong></summary>

```bash
# Fix X11 permissions
xhost +local:docker

# Or reset X11 auth
sudo rm -rf /tmp/.docker.xauth
```
</details>

<details>
<summary><strong>🎮 GPU Not Detected in Container</strong></summary>

```bash
# Check NVIDIA runtime is available
sudo docker info | grep nvidia

# If not found, reinstall NVIDIA container toolkit
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```
</details>

<details>
<summary><strong>🔒 Permission Denied Errors</strong></summary>

```bash
# Make scripts executable
chmod +x run2.bash run_no_nvidia.bash

# Add user to docker group (requires logout/login)
sudo usermod -aG docker $USER
```
</details>

<details>
<summary><strong>📦 Container Already Exists</strong></summary>

```bash
# Scripts auto-clean, but manual cleanup if needed
sudo docker rm -f ros2os ros2os_no_nvidia
```
</details>

---

## ⚡ Performance Tips

### 🚀 **Maximize GPU Performance**
- **Use** `sudo ./run2.bash` for GPU acceleration
- **Monitor** GPU usage with `nvidia-smi` inside container
- **Close** unnecessary GUI applications to free GPU memory

### 💾 **Container Management**
- **Rebuild** container when updating packages: `sudo docker build -t shriman1:a .`
- **Clean up** old containers: `sudo docker system prune`
- **Save** your work outside container (containers are ephemeral)

---

## 🎯 Next Steps

### 🏗️ **Development Workflow**
1. **Start container**: `sudo ./run2.bash`
2. **Source ROS2**: `source /opt/ros/humble/setup.bash`
3. **Create workspace**: `mkdir -p ~/ros2_ws/src && cd ~/ros2_ws`
4. **Develop your nodes**: Create packages with `ros2 pkg create`
5. **Build & test**: `colcon build && source install/setup.bash`

### 📚 **Learning Resources**
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [Gazebo Documentation](http://gazebosim.org/tutorials)
- [RViz2 User Guide](https://github.com/ros2/rviz)

---

## 🎉 You're All Set!

Your ROS2 development environment is ready with:
- ✅ **Full GPU acceleration** (RTX 4070 Ti)
- ✅ **Complete ROS2 Humble** installation
- ✅ **GUI applications** working perfectly
- ✅ **Hardware integration** capabilities
- ✅ **Arduino development** tools included

**Happy Robot Building!** 🤖✨

---

<details>
<summary><strong>📋 Technical Details</strong></summary>

### Container Configuration
- **Base**: osrf/ros:humble-desktop
- **GPU**: NVIDIA runtime with all capabilities
- **Network**: Host mode for easy ROS communication
- **Privileges**: Full hardware access for sensors/actuators
- **Display**: X11 forwarding for GUI applications

### Environment Variables
- `NVIDIA_VISIBLE_DEVICES=all`
- `NVIDIA_DRIVER_CAPABILITIES=graphics`
- `DISPLAY=$DISPLAY`
- `QT_X11_NO_MITSHM=1`

</details>