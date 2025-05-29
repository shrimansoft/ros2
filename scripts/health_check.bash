#!/bin/bash

# Health check script for ROS2 Bag Annotator
# Verifies Docker, NVIDIA, X11, and application components

set -e

echo "🔍 ROS2 Bag Annotator Health Check"
echo "=================================="
echo ""

# Check Docker
echo "🐳 Docker Status:"
if command -v docker &> /dev/null; then
    docker --version
    if docker info &> /dev/null; then
        echo "✅ Docker is running"
    else
        echo "❌ Docker daemon is not running"
        exit 1
    fi
else
    echo "❌ Docker is not installed"
    exit 1
fi
echo ""

# Check Docker image
echo "🖼️  Docker Image:"
if docker images | grep -q "shriman1:a"; then
    echo "✅ shriman1:a image found"
    docker images shriman1:a --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}\t{{.CreatedSince}}"
else
    echo "⚠️  shriman1:a image not found - run 'make build'"
fi
echo ""

# Check NVIDIA Docker support
echo "🎮 NVIDIA GPU Support:"
if docker info 2>/dev/null | grep -q nvidia; then
    echo "✅ NVIDIA Docker runtime available"
    if command -v nvidia-smi &> /dev/null; then
        nvidia-smi --query-gpu=name,memory.total --format=csv,noheader,nounits | head -1
    fi
else
    echo "⚠️  NVIDIA Docker runtime not available"
fi
echo ""

# Check X11
echo "🖥️  Display Support:"
if [ -n "$DISPLAY" ]; then
    echo "✅ DISPLAY set to: $DISPLAY"
    if [ -d "/tmp/.X11-unix" ]; then
        echo "✅ X11 socket directory exists"
    else
        echo "⚠️  X11 socket directory not found"
    fi
else
    echo "⚠️  DISPLAY not set"
fi
echo ""

# Check data directory
echo "📁 Data Directory:"
if [ -d "rosbag_annotator/data" ]; then
    size=$(du -sh rosbag_annotator/data 2>/dev/null | cut -f1)
    files=$(find rosbag_annotator/data -type f 2>/dev/null | wc -l)
    echo "✅ Data directory exists: $size in $files files"
else
    echo "⚠️  Data directory not found (will be created automatically)"
fi
echo ""

# Test container startup
echo "🧪 Container Test:"
if docker run --rm shriman1:a echo "Container test successful" 2>/dev/null; then
    echo "✅ Container can start successfully"
else
    echo "❌ Container startup failed"
    exit 1
fi
echo ""

echo "🎉 Health check completed!"
echo ""
echo "Next steps:"
echo "  - Run 'make annotate' to start the annotation GUI"
echo "  - Run 'make info' to see project details"
echo "  - Check 'make help' for all available commands"
