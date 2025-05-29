# 🛠️ Setup Instructions

## Prerequisites

### System Requirements
- **OS**: Linux (Ubuntu 20.04+ recommended)
- **Docker**: Version 20.10+
- **NVIDIA GPU**: Optional but recommended for acceleration
- **Display**: X11 support for GUI applications

### Required Packages
```bash
# Install Docker (if not already installed)
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER

# Install NVIDIA Docker support (for GPU acceleration)
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update && sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

## Quick Setup

### 1. Clone and Build
```bash
git clone <your-repo-url>
cd ros2-bag-annotator
make setup
```

### 2. Verify Installation
```bash
make health
```

### 3. Add Sample Data
```bash
# Copy your bag files to the data directory
./scripts/bag_utils.bash copy /path/to/your/bag.db3

# Or manually copy
cp -r /path/to/your/bag/ ./rosbag_annotator/data/
```

### 4. Start Annotating
```bash
make annotate
```

## Troubleshooting

### Docker Issues
```bash
# Check Docker status
docker info

# Test container
docker run --rm hello-world
```

### GPU Issues
```bash
# Test NVIDIA support
nvidia-smi
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
```

### Display Issues
```bash
# Check X11
echo $DISPLAY
xhost +local:docker  # Allow Docker to access display
```

### Permission Issues
```bash
# Fix Docker permissions
sudo usermod -aG docker $USER
# Log out and back in

# Fix file permissions
chmod +x scripts/*.bash
chmod +x run_annotator.bash
```

## Advanced Setup

### Development Environment
```bash
make dev  # Start development container with source mounting
```

### Custom Configuration
Edit `rosbag_annotator/config/annotator_config.yaml` to customize:
- Annotation types
- Video display settings
- Export preferences

### Data Management
```bash
./scripts/bag_utils.bash help  # See all bag management commands
```
