# ROS2 Bag Annotator - Deployment Guide

This document provides comprehensive deployment options for the ROS2 Bag Annotator.

## 🚀 Quick Start

```bash
# 1. Clone or navigate to the repository
cd /path/to/ros2-bag-annotator

# 2. Quick setup and launch
make quick

# 3. Start annotation GUI
make annotate
```

## 🐳 Deployment Options

### Option 1: Docker (Recommended)

**Single Command Deployment:**
```bash
make setup    # Build image and run health check
make annotate # Launch GUI
```

**Manual Docker Commands:**
```bash
# Build image
docker build -t shriman1:a .

# Run with GUI
./run_annotator.bash annotate

# Interactive shell
./run_annotator.bash bash
```

### Option 2: Docker Compose

**GUI Service:**
```bash
# Start annotation GUI
docker compose up annotator-gui

# Development environment
docker compose run --rm ros2-annotator bash

# Stop all services
docker compose down
```

**Background Services:**
```bash
# Start all services in background
docker compose up -d

# View logs
docker compose logs -f

# Scale services
docker compose up --scale ros2-annotator=2
```

### Option 3: Development Mode

**Live Development:**
```bash
# Development container with live code mounting
make dev

# Or with Docker Compose
docker compose run --rm ros2-annotator bash
```

## 📁 Data Management

### Data Directories

The system uses these data directories:
- `./data/` - Primary data directory (automatically created)
- `./rosbag_annotator/data/` - Legacy data directory
- `./logs/` - Application logs
- `./rosbag_annotator/config/` - Configuration files

### Bag File Management

```bash
# List bag files
make bags

# Copy bag files
./scripts/bag_utils.bash copy /path/to/your/bag

# Info about specific bag
./scripts/bag_utils.bash info /data/my_bag

# Clean up old files
./scripts/bag_utils.bash clean
```

## 🔧 Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `DISPLAY` | `:1` | X11 display for GUI |
| `NVIDIA_VISIBLE_DEVICES` | `all` | GPU devices to expose |
| `QT_X11_NO_MITSHM` | `1` | X11 compatibility |

### Docker Customization

**Custom Data Location:**
```bash
# Mount custom data directory
docker run -it \
  -v /your/data/path:/data \
  shriman1:a annotate
```

**GPU Configuration:**
```bash
# Specific GPU
docker run --gpus device=0 shriman1:a annotate

# All GPUs
docker run --gpus all shriman1:a annotate
```

## 🌐 Network Deployment

### Remote Access

**X11 Forwarding over SSH:**
```bash
# On remote machine
ssh -X user@remote-host
cd /path/to/annotator
make annotate
```

**VNC Server (for headless systems):**
```bash
# Install VNC server on host
sudo apt install tigervnc-standalone-server

# Start VNC session
vncserver :1

# Set DISPLAY and run
export DISPLAY=:1
make annotate
```

### Web Interface (Future Enhancement)

The repository is structured to support future web-based interfaces:
- `docs/API.md` - Documents potential REST API
- `tests/integration/` - Includes web service testing framework

## 🧪 Testing & Validation

### Local Testing

```bash
# Run all tests
make test

# Test specific components
make test-unit          # Unit tests
make test-integration   # Integration tests
make test-docker        # Container tests
```

### Production Validation

```bash
# System health check
make health

# Validate configuration
make validate

# Project status
make status
```

## 📊 Monitoring & Debugging

### Log Files

Application logs are stored in `./logs/`:
- `annotator_debug_*.log` - Debug information
- `annotator_errors_*.log` - Error messages

### Container Debugging

```bash
# Enter running container
docker exec -it ros2_annotator bash

# View container logs
docker logs ros2_annotator

# Monitor resource usage
docker stats ros2_annotator
```

### Performance Monitoring

```bash
# GPU usage (if available)
nvidia-smi

# System resources
htop

# Container resources
docker compose top
```

## 🔄 Updates & Maintenance

### Updating the Application

```bash
# Rebuild with latest changes
make rebuild

# Clean and rebuild everything
make clean && make build

# Update dependencies
make install-dev
```

### Backup & Recovery

```bash
# Backup data and config
tar -czf backup.tar.gz data/ rosbag_annotator/config/ logs/

# Restore from backup
tar -xzf backup.tar.gz
```

## 🚨 Troubleshooting

### Common Issues

**GUI Not Starting:**
1. Check X11 forwarding: `echo $DISPLAY`
2. Test X11: `xhost +local:docker`
3. Verify GPU access: `nvidia-smi`

**Container Issues:**
1. Rebuild image: `make rebuild`
2. Clean containers: `make clean`
3. Check logs: `docker logs ros2_annotator`

**Performance Issues:**
1. Check GPU usage: `nvidia-smi`
2. Monitor memory: `docker stats`
3. Optimize data location: use SSD for `/data`

### Getting Help

1. **Documentation:** Check `docs/` directory
2. **Logs:** Review `./logs/` files
3. **Health Check:** Run `make health`
4. **Validation:** Run `make validate`

## 📈 Production Deployment

### System Requirements

**Minimum:**
- 4GB RAM
- 2 CPU cores
- 10GB disk space
- Docker 20.10+

**Recommended:**
- 8GB RAM
- 4+ CPU cores
- 50GB SSD storage
- NVIDIA GPU (RTX series)
- Ubuntu 20.04+ or equivalent

### Security Considerations

1. **Container Security:**
   - Run with non-root user when possible
   - Limit container capabilities
   - Use read-only mounts for code

2. **Data Security:**
   - Encrypt sensitive bag files
   - Use proper file permissions
   - Regular backups

3. **Network Security:**
   - Firewall configuration for remote access
   - VPN for production deployments
   - SSL/TLS for web interfaces

### Scaling

For multiple users or large datasets:

1. **Horizontal Scaling:**
   ```bash
   # Multiple annotation instances
   docker compose up --scale ros2-annotator=3
   ```

2. **Load Balancing:**
   - Use nginx for web interface load balancing
   - Shared storage for bag files
   - Database for annotation metadata

3. **Cloud Deployment:**
   - AWS ECS/EKS with GPU instances
   - Google Cloud Run with GPU support
   - Azure Container Instances

---

*This deployment guide covers various scenarios from development to production. Choose the deployment method that best fits your use case and infrastructure.*
