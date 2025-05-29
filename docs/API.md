# 🔧 API Documentation

## Python Modules

### AnnotationGUI (`annotation_gui.py`)

Main GUI application for bag annotation.

#### Classes

##### `AnnotationMessage`
Custom message for storing annotations.

```python
class AnnotationMessage:
    def __init__(self, timestamp, annotation_type, description, confidence):
        self.timestamp = timestamp
        self.annotation_type = annotation_type  
        self.description = description
        self.confidence = confidence
```

##### `BagAnnotatorGUI`
Main GUI application class.

```python
class BagAnnotatorGUI:
    def __init__(self):
        # Initialize GUI components and ROS2 node
        
    def load_bag_file(self, bag_path: str) -> bool:
        # Load and process bag file
        
    def add_annotation(self, timestamp: float, 
                      annotation_type: str, 
                      description: str, 
                      confidence: float) -> None:
        # Add new annotation
        
    def export_annotated_bag(self, output_path: str) -> bool:
        # Export bag with embedded annotations
```

#### Key Methods

##### Bag Operations
- `load_bag_file(bag_path)` - Load bag for annotation
- `get_bag_info()` - Extract bag metadata
- `get_topics_info()` - List available topics

##### Annotation Management  
- `add_annotation()` - Create new annotation
- `edit_annotation()` - Modify existing annotation
- `delete_annotation()` - Remove annotation
- `get_annotations_at_time()` - Get annotations near timestamp

##### Playback Control
- `play_bag()` - Start playback
- `pause_bag()` - Pause playback  
- `seek_to_time()` - Jump to specific timestamp
- `set_playback_speed()` - Change playback rate

##### Export Functions
- `export_annotated_bag()` - Create annotated bag file
- `export_annotations_csv()` - Export as CSV
- `export_annotations_json()` - Export as JSON

### BagPlayer (`bag_player.py`)

ROS2 bag playback functionality.

#### Classes

##### `BagPlayer`
Handles bag file reading and message playback.

```python
class BagPlayer:
    def __init__(self, bag_path: str):
        self.bag_path = bag_path
        
    def get_topics(self) -> List[str]:
        # Get list of topics in bag
        
    def get_messages(self, topic: str) -> Iterator[Tuple[float, Any]]:
        # Iterate through messages for topic
        
    def get_duration(self) -> float:
        # Get total bag duration
```

### AnnotationMerger (`annotation_merger.py`)

Merges annotations back into bag files.

#### Classes

##### `AnnotationMerger`
Combines original bag with annotation data.

```python
class AnnotationMerger:
    def __init__(self, original_bag: str, annotations: List[AnnotationMessage]):
        # Initialize merger
        
    def merge(self, output_path: str) -> bool:
        # Create merged bag file
```

## Configuration

### `annotator_config.yaml`

Main configuration file structure:

```yaml
annotation_types:
  safety: "Safe driving conditions"
  danger: "Dangerous situations"
  # ... more types

defaults:
  confidence: 1.0
  annotation_type: "comment"

video:
  max_width: 640
  max_height: 480
  fps: 30

export:
  annotation_topic: "/ev_annotations"
  include_original_topics: true
```

#### Configuration Options

##### Annotation Types
Define custom annotation categories:
```yaml
annotation_types:
  custom_type: "Description for UI"
```

##### Video Settings
Control video display:
```yaml
video:
  max_width: 1280      # Max video width
  max_height: 720      # Max video height  
  fps: 30              # Target framerate
  auto_resize: true    # Auto-fit to window
```

##### Export Settings
Configure output behavior:
```yaml
export:
  annotation_topic: "/annotations"
  timestamp_format: "ros_time"
  include_metadata: true
```

## ROS2 Interface

### Topics

#### Published Topics
- `/ev_annotations` - Annotation messages during playback
- `/annotation_status` - Current annotation system status

#### Subscribed Topics  
- `/camera/image_raw` - Primary video stream
- `/camera/image_raw/compressed` - Compressed video
- Custom image topics (configurable)

### Messages

#### Custom Annotation Message
```python
# annotation_msg.py
class AnnotationMsg:
    timestamp: float
    annotation_type: string
    description: string  
    confidence: float
    frame_id: string
```

### Services

#### Available Services
- `get_bag_info` - Retrieve bag metadata
- `export_annotations` - Trigger annotation export
- `validate_bag` - Check bag file integrity

## Docker Interface

### Environment Variables

- `DISPLAY` - X11 display for GUI
- `ROS_DOMAIN_ID` - ROS2 domain (default: 0)
- `ANNOTATION_CONFIG` - Custom config file path

### Volume Mounts

- `/data` - Bag files and output
- `/logs` - Application logs  
- `/config` - Configuration files

### Network Configuration

- `--net=host` - Host networking for ROS2
- GPU access via `--gpus all`
- X11 forwarding via volume mounts

## Extension Points

### Custom Annotation Types

Add new types in `annotator_config.yaml`:
```yaml
annotation_types:
  my_custom_type: "My custom annotation"
```

### Custom Video Processing

Override video processing in `annotation_gui.py`:
```python
def process_image_message(self, msg):
    # Custom image processing logic
    return processed_image
```

### Custom Export Formats

Extend export functionality:
```python
def export_custom_format(self, annotations, output_path):
    # Custom export implementation
    pass
```

## Error Handling

### Common Error Codes

- `BAG_NOT_FOUND` - Bag file doesn't exist
- `INVALID_BAG_FORMAT` - Corrupted or invalid bag
- `NO_IMAGE_TOPICS` - No compatible video topics
- `EXPORT_FAILED` - Export operation failed
- `PERMISSION_DENIED` - File access issues

### Debug Information

Enable debug logging:
```bash
./run_annotator.bash --debug annotate
```

Log files location:
- `logs/annotator_debug_*.log` - Detailed operation logs
- `logs/annotator_errors_*.log` - Error messages only
