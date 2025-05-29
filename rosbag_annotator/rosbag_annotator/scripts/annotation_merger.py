#!/usr/bin/env python3

import sys
import os
import json
import sqlite3
import yaml
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import String, Header
from builtin_interfaces.msg import Time

class AnnotationMerger(Node):
    def __init__(self):
        super().__init__('annotation_merger')
        
    def create_annotated_bag(self, input_bag_path: str, annotations_file: str, output_bag_path: str):
        """Create a new bag file with annotations added as a new topic"""
        
        try:
            # Load annotations
            with open(annotations_file, 'r') as f:
                annotations = json.load(f)
            
            self.get_logger().info(f"Loaded {len(annotations)} annotations")
            
            # Copy original bag structure
            self._copy_bag_structure(input_bag_path, output_bag_path)
            
            # Add annotations to the new bag
            self._add_annotations_to_bag(output_bag_path, annotations)
            
            self.get_logger().info(f"Created annotated bag at {output_bag_path}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to create annotated bag: {e}")
            raise
    
    def _copy_bag_structure(self, input_path: str, output_path: str):
        """Copy the original bag structure to output path"""
        import shutil
        
        # Create output directory
        os.makedirs(output_path, exist_ok=True)
        
        # Copy database file
        input_db = os.path.join(input_path, 'rosbag2_2025_03_27-09_44_20_0.db3')
        output_db = os.path.join(output_path, 'rosbag2_annotated.db3')
        shutil.copy2(input_db, output_db)
        
        # Copy and modify metadata
        input_metadata = os.path.join(input_path, 'metadata.yaml')
        output_metadata = os.path.join(output_path, 'metadata.yaml')
        
        with open(input_metadata, 'r') as f:
            metadata = yaml.safe_load(f)
        
        # Add annotation topic to metadata
        annotation_topic = {
            'topic_metadata': {
                'name': '/ev_annotations',
                'type': 'std_msgs/msg/String',
                'serialization_format': 'cdr',
                'offered_qos_profiles': '''- history: 3
  depth: 0
  reliability: 1
  durability: 2
  deadline:
    sec: 9223372036
    nsec: 854775807
  lifespan:
    sec: 9223372036
    nsec: 854775807
  liveliness: 1
  liveliness_lease_duration:
    sec: 9223372036
    nsec: 854775807
  avoid_ros_namespace_conventions: false'''
            },
            'message_count': 0  # Will be updated later
        }
        
        metadata['rosbag2_bagfile_information']['topics_with_message_count'].append(annotation_topic)
        metadata['rosbag2_bagfile_information']['relative_file_paths'] = ['rosbag2_annotated.db3']
        metadata['rosbag2_bagfile_information']['files'][0]['path'] = 'rosbag2_annotated.db3'
        
        with open(output_metadata, 'w') as f:
            yaml.dump(metadata, f, default_flow_style=False)
    
    def _add_annotations_to_bag(self, output_path: str, annotations: list):
        """Add annotations as messages to the bag database"""
        
        db_path = os.path.join(output_path, 'rosbag2_annotated.db3')
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        
        try:
            # Add annotation topic to topics table
            cursor.execute("""
                INSERT INTO topics (id, name, type, serialization_format, offered_qos_profiles)
                VALUES (?, ?, ?, ?, ?)
            """, (
                999,  # Use a high ID that shouldn't conflict
                '/ev_annotations',
                'std_msgs/msg/String',
                'cdr',
                '''- history: 3
  depth: 0
  reliability: 1
  durability: 2
  deadline:
    sec: 9223372036
    nsec: 854775807
  lifespan:
    sec: 9223372036
    nsec: 854775807
  liveliness: 1
  liveliness_lease_duration:
    sec: 9223372036
    nsec: 854775807
  avoid_ros_namespace_conventions: false'''
            ))
            
            # Add annotation messages
            message_count = 0
            for annotation in annotations:
                # Create ROS message
                msg = String()
                
                # Create structured annotation data
                annotation_data = {
                    'id': annotation['annotation_id'],
                    'type': annotation['type'],
                    'text': annotation['text'],
                    'confidence': annotation['confidence'],
                    'datetime': annotation['datetime']
                }
                
                msg.data = json.dumps(annotation_data)
                
                # Serialize message
                serialized_msg = serialize_message(msg)
                
                # Insert into messages table
                cursor.execute("""
                    INSERT INTO messages (topic_id, timestamp, data)
                    VALUES (?, ?, ?)
                """, (999, annotation['timestamp'], serialized_msg))
                
                message_count += 1
            
            # Update message count in metadata
            self._update_metadata_message_count(output_path, message_count)
            
            conn.commit()
            self.get_logger().info(f"Added {message_count} annotation messages to bag")
            
        except Exception as e:
            conn.rollback()
            self.get_logger().error(f"Error adding annotations: {e}")
            raise
        finally:
            conn.close()
    
    def _update_metadata_message_count(self, output_path: str, message_count: int):
        """Update the message count in metadata.yaml"""
        metadata_path = os.path.join(output_path, 'metadata.yaml')
        
        with open(metadata_path, 'r') as f:
            metadata = yaml.safe_load(f)
        
        # Update annotation topic message count
        topics = metadata['rosbag2_bagfile_information']['topics_with_message_count']
        for topic in topics:
            if topic['topic_metadata']['name'] == '/ev_annotations':
                topic['message_count'] = message_count
                break
        
        # Update total message count
        metadata['rosbag2_bagfile_information']['message_count'] += message_count
        
        with open(metadata_path, 'w') as f:
            yaml.dump(metadata, f, default_flow_style=False)

def main():
    rclpy.init()
    
    try:
        merger = AnnotationMerger()
        
        if len(sys.argv) != 4:
            print("Usage: annotation_merger.py <input_bag_path> <annotations_file> <output_bag_path>")
            sys.exit(1)
        
        input_bag_path = sys.argv[1]
        annotations_file = sys.argv[2]
        output_bag_path = sys.argv[3]
        
        merger.create_annotated_bag(input_bag_path, annotations_file, output_bag_path)
        print(f"Successfully created annotated bag at {output_bag_path}")
        
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
