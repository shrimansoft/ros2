# 📖 Usage Guide

## Getting Started

### 1. Launch the Annotation GUI
```bash
make annotate
```

### 2. Load a Bag File
1. Click **"Browse"** button
2. Select a bag file from the dropdown
3. Click **"Load Bag"**

### 3. Navigate and Annotate
- **Play/Pause**: Control playback
- **Seek**: Click on timeline to jump to specific time
- **Speed**: Adjust playback speed (0.1x to 4.0x)
- **Timeline**: Visual representation of bag duration

### 4. Add Annotations
1. **Pause** at desired timestamp
2. **Click** on the timeline or use current position
3. **Select** annotation type from dropdown
4. **Enter** description text
5. **Set** confidence level (0.0 to 1.0)
6. **Click** "Add Annotation"

## Annotation Types

| Type | Purpose | Example Use |
|------|---------|-------------|
| `safety` | Safe driving conditions | "Clear road, good visibility" |
| `danger` | Hazardous situations | "Obstacle ahead, pedestrian crossing" |
| `comment` | General observations | "Interesting sensor behavior" |
| `traffic_sign` | Traffic signs and signals | "Stop sign detected" |
| `pedestrian` | Pedestrian activity | "Pedestrian on sidewalk" |
| `vehicle` | Vehicle interactions | "Car merging from left" |
| `road_condition` | Road surface conditions | "Wet road surface" |
| `weather` | Weather conditions | "Light rain beginning" |

## Video Display

### Supported Topics
- `/camera/image_raw` (uncompressed)
- `/camera/image_raw/compressed` (compressed)
- Most standard ROS2 image topics

### Display Features
- **Auto-resize**: Maintains aspect ratio
- **Frame info**: Shows timestamp and frame details
- **Sync**: Video synchronized with bag playback

## Managing Annotations

### View Annotations
- **Timeline markers**: Visual indicators on timeline
- **Annotation list**: Detailed list with timestamps
- **Edit**: Double-click to modify existing annotations
- **Delete**: Select and press Delete key

### Export Options
```bash
# Export annotated bag with embedded annotations
# Available in GUI: File → Export Annotated Bag
```

## Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `Space` | Play/Pause |
| `Left Arrow` | Step backward |
| `Right Arrow` | Step forward |
| `Home` | Go to beginning |
| `End` | Go to end |
| `+` | Increase speed |
| `-` | Decrease speed |
| `Ctrl+A` | Add annotation at current time |
| `Delete` | Delete selected annotation |

## Command Line Tools

### Bag Management
```bash
# List available bags
./scripts/bag_utils.bash list

# Show bag information
./scripts/bag_utils.bash info my_bag

# Copy new bag
./scripts/bag_utils.bash copy /path/to/bag

# Clean all bags
./scripts/bag_utils.bash clean
```

### System Health
```bash
# Check system status
make health

# Show project info
make info

# Check container status
make status
```

## Tips and Best Practices

### Efficient Annotation Workflow
1. **Preview first**: Play through the entire bag to understand content
2. **Use consistent types**: Stick to defined annotation categories
3. **Add context**: Include relevant details in descriptions
4. **Set confidence**: Use confidence levels to indicate certainty
5. **Regular saves**: Export periodically to avoid data loss

### Performance Optimization
- **Reduce video size**: Large images may slow playback
- **Close unused apps**: Free up system resources
- **Use SSD storage**: Faster bag file access
- **GPU acceleration**: Enables smoother video rendering

### Data Organization
```
data/
├── original_bags/     # Source bag files
├── in_progress/       # Currently being annotated
├── completed/         # Finished annotations
└── exported/          # Final annotated bags
```

## Troubleshooting

### Common Issues

#### GUI Won't Start
```bash
# Check X11 forwarding
echo $DISPLAY
xhost +local:docker

# Check container logs
docker logs ros2_annotator
```

#### No Video Display
- Verify image topics exist in bag
- Check topic names in bag info
- Ensure proper image encoding

#### Slow Performance
- Reduce video resolution in config
- Close other applications
- Check available system memory

#### Export Fails
- Verify write permissions in data directory
- Check available disk space
- Ensure bag file is not corrupted

### Debug Mode
```bash
# Run with detailed logging
./run_annotator.bash --debug annotate

# Check logs
cat logs/annotator_debug_*.log
cat logs/annotator_errors_*.log
```
