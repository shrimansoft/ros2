#!/bin/bash

# Quick annotate script for new users
# Provides a guided setup and annotation workflow

set -e

echo "🚀 ROS2 Bag Annotator - Quick Start"
echo "===================================="
echo ""

# Check if Docker image exists
if ! docker images | grep -q "shriman1:a"; then
    echo "📦 Docker image not found. Building now..."
    make build
    echo ""
fi

# Run health check
echo "🔍 Running health check..."
./scripts/health_check.bash
echo ""

# Check for bag files
if [ ! -d "./rosbag_annotator/data" ] || [ -z "$(find ./rosbag_annotator/data -name "*.db3" 2>/dev/null)" ]; then
    echo "📁 No bag files found in data directory."
    echo ""
    echo "Please add some bag files first:"
    echo "  1. Copy your .bag files to: ./rosbag_annotator/data/"
    echo "  2. Or use: ./scripts/bag_utils.bash copy /path/to/your/bag"
    echo ""
    read -p "Do you want to see available commands? (y/N): " show_help
    if [ "$show_help" = "y" ] || [ "$show_help" = "Y" ]; then
        echo ""
        make help
    fi
    exit 0
fi

# Show available bags
echo "📊 Available bag files:"
./scripts/bag_utils.bash list
echo ""

# Ask user if they want to proceed
read -p "Ready to start annotation GUI? (Y/n): " start_gui

if [ "$start_gui" = "n" ] || [ "$start_gui" = "N" ]; then
    echo ""
    echo "💡 When you're ready:"
    echo "  make annotate    # Start annotation GUI"
    echo "  make help        # See all commands"
    exit 0
fi

# Launch annotation GUI
echo ""
echo "🎯 Launching annotation GUI..."
echo "   - Load a bag file from the dropdown"
echo "   - Click 'Play' to start playback"
echo "   - Click anywhere on the timeline to add annotations"
echo "   - Use 'Export' to save annotated bags"
echo ""

make annotate
