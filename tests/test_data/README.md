# Test Data Directory

This directory contains sample data for testing the ROS2 Bag Annotator.

## Contents

### Sample Bag Files
- `sample_bag/` - Minimal test bag for basic functionality
- `annotated_sample/` - Pre-annotated bag for testing export features

### Test Configurations
- `test_config.yaml` - Test configuration with minimal settings

## Creating Test Data

### Generate Test Bag
```bash
# Create a simple test bag with ROS2
ros2 bag record -o test_bag /your_topic

# Or use the provided sample
cp -r sample_bag/ your_test_name/
```

### Test Annotation Data
```json
[
    {
        "timestamp": 1.5,
        "type": "safety",
        "description": "Test annotation 1",
        "confidence": 0.9
    },
    {
        "timestamp": 3.2,
        "type": "comment", 
        "description": "Test annotation 2",
        "confidence": 1.0
    }
]
```

## Usage in Tests

Tests automatically use data from this directory:
```python
test_data_dir = Path(__file__).parent / "test_data"
sample_bag = test_data_dir / "sample_bag"
```

## Guidelines

- Keep test data small (< 10MB total)
- Use meaningful names for test scenarios
- Document any special requirements
- Include both valid and invalid test cases
