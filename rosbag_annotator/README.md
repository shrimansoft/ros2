# ROS2 Bag Annotator

A comprehensive tool for annotating ROS2 bag files containing Electric Vehicle (EV) sensor data. This software allows you to play back video streams from bag files, pause at specific moments, and add text-based annotations that are meaningful for EV applications.

## Features

- **Video Playback**: Play, pause, and navigate through compressed image topics in ROS2 bag files
- **Manual Annotation**: Add text annotations with different categories (safety, danger, comments, etc.)
- **Real-time Preview**: See video frames while annotating with playback controls
- **Annotation Export**: Create new bag files with annotation data as an additional topic
- **Flexible Categories**: Support for various annotation types relevant to EV applications
- **Time-synchronized**: All annotations are precisely time-stamped with the original data

## Supported Data Types

The tool currently supports bag files containing:
- `/image_raw/compressed` - Compressed video streams
- `/camera_info` - Camera calibration information  
- `/imu/data` - IMU sensor data
- `/timestamp` - Timestamp information

## Installation

### Prerequisites

- ROS2 (Humble or later)
- Python 3.8+
- OpenCV
- Tkinter (usually included with Python)

### Setup

1. Clone or copy this package to your ROS2 workspace
2. Run the setup script:
   ```bash
   cd rosbag_annotator
   ./setup.sh
   ```

3. Source your workspace:
   ```bash
   source ../install/setup.bash
   ```

## Usage

### 1. GUI-based Annotation

Launch the annotation GUI:
```bash
ros2 run rosbag_annotator annotation_gui.py
```

Or using launch file:
```bash
ros2 launch rosbag_annotator annotation_gui.launch.py
```

#### GUI Features:
- **Load Bag File**: Browse and select your ROS2 bag directory
- **Playback Controls**: Play, pause, previous/next frame navigation
- **Speed Control**: Adjust playback speed (0.1x to 3.0x)
- **Progress Bar**: Seek to any position in the video
- **Annotation Panel**: 
  - Select annotation type (safety, danger, comment, etc.)
  - Enter descriptive text
  - Set confidence level
  - View all annotations in a list
- **Export**: Save annotations and create annotated bag files

#### Annotation Types:
- **Safety**: Safe driving conditions
- **Danger**: Dangerous situations (dengue areas, hazards)
- **Comment**: General observations
- **Traffic Sign**: Traffic signs and signals
- **Pedestrian**: Pedestrian activity
- **Vehicle**: Vehicle interactions
- **Road Condition**: Road surface conditions
- **Weather**: Weather conditions

### 2. Playing Annotated Bag Files

To play back an annotated bag file:
```bash
ros2 run rosbag_annotator bag_player.py /path/to/annotated/bag [playback_rate]
```

This will publish both the original topics and the new `/ev_annotations` topic.

### 3. Creating Annotated Bag Files

To merge annotations into a new bag file:
```bash
ros2 run rosbag_annotator annotation_merger.py /path/to/original/bag annotations.json /path/to/output/bag
```

## File Structure

```
rosbag_annotator/
├── scripts/
│   ├── annotation_gui.py      # Main GUI application
│   ├── bag_player.py          # Bag file player
│   └── annotation_merger.py   # Merge annotations into bag
├── launch/
│   ├── annotation_gui.launch.py
│   └── bag_player.launch.py
├── config/
│   └── annotator_config.yaml  # Configuration settings
├── data/                      # Your bag files go here
└── README.md
```

## Annotation Data Format

Annotations are stored in JSON format with the following structure:

```json
{
  "timestamp": 1743048861434914422,
  "annotation_id": "ann_0001",
  "type": "danger",
  "text": "Dengue risk area - standing water visible",
  "confidence": 0.9,
  "datetime": "2025-03-27T09:44:21.434"
}
```

When exported to a bag file, annotations become ROS2 messages on the `/ev_annotations` topic.

## Workflow Example

1. **Load your bag file** containing EV sensor data
2. **Navigate through the video** using playback controls
3. **Pause at interesting moments** (dangers, safe areas, notable events)
4. **Add annotations** with appropriate categories and descriptions
5. **Save annotations** to JSON file for backup
6. **Export annotated bag** for sharing or further analysis

## Use Cases

- **Safety Assessment**: Mark safe vs dangerous driving areas
- **Environmental Monitoring**: Note weather, road conditions
- **Traffic Analysis**: Identify pedestrians, vehicles, traffic signs
- **Route Planning**: Document route characteristics for autonomous systems
- **Training Data**: Create labeled datasets for machine learning

## Configuration

Edit `config/annotator_config.yaml` to customize:
- Available annotation types
- Default settings
- Video display parameters
- Export options

## Troubleshooting

### Common Issues:

1. **"No video loaded"**: Ensure your bag file contains `/image_raw/compressed` topic
2. **Build errors**: Make sure all ROS2 dependencies are installed
3. **GUI not appearing**: Check that Tkinter is properly installed
4. **Playback issues**: Verify the bag file is not corrupted

### Getting Help:

- Check the console output for detailed error messages
- Ensure your ROS2 environment is properly sourced
- Verify bag file format using `ros2 bag info /path/to/bag`

## Contributing

This tool was designed specifically for EV data annotation but can be adapted for other robotics applications. Feel free to extend the annotation categories or add new features as needed.

## License

MIT License - Feel free to modify and distribute as needed for your EV data analysis projects.
