#!/usr/bin/env python3

import sys
import os
import json
import sqlite3
import cv2
import numpy as np
import yaml
import tempfile
import subprocess
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Tuple
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CompressedImage, CameraInfo, Image as SensorImage
from std_msgs.msg import String
from std_msgs.msg import String, Header
from geometry_msgs.msg import Vector3Stamped
from cv_bridge import CvBridge

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from PIL import Image, ImageTk

class AnnotationMessage:
    """Custom message for annotations"""
    def __init__(self):
        self.header = Header()
        self.annotation_id = ""
        self.annotation_type = ""  # "safety", "danger", "comment"
        self.text = ""
        self.confidence = 1.0
        self.location = Vector3Stamped()  # Optional GPS location

class RosbagAnnotator(Node):
    def __init__(self):
        super().__init__('rosbag_annotator')
        self.bridge = CvBridge()
        
        # Data storage
        self.annotations = []
        self.current_bag_path = None
        self.current_frame_time = None
        self.total_duration = 0
        
        # Playback control
        self.is_playing = False
        self.playback_speed = 1.0
        self.current_position = 0.0
        
        # Dynamic topic storage
        self.available_topics = {}  # topic_name -> topic_info
        self.video_topics = {}      # topic_name -> list of (timestamp, message)
        self.other_topics = {}      # topic_name -> list of (timestamp, message)
        self.selected_video_topics = []  # Topics selected for video display
        
        # Legacy support
        self.image_messages = []  # Will point to first video topic for backward compatibility
        self.timestamp_messages = []
        self.camera_info_messages = []
        self.imu_messages = []
        
    def load_bag_file(self, bag_path: str) -> bool:
        """Load a ROS2 bag file and extract messages"""
        try:
            self.current_bag_path = bag_path
            
            # Clear previous data
            self.available_topics.clear()
            self.video_topics.clear()
            self.other_topics.clear()
            self.image_messages.clear()
            
            # First, read metadata to get topic information
            metadata_path = os.path.join(bag_path, 'metadata.yaml')
            if os.path.exists(metadata_path):
                with open(metadata_path, 'r') as f:
                    metadata = yaml.safe_load(f)
                self._parse_metadata(metadata)
            
            # Find the database file
            db_files = [f for f in os.listdir(bag_path) if f.endswith('.db3')]
            if not db_files:
                self.get_logger().error("No database file found in bag directory")
                return False
            
            db_path = os.path.join(bag_path, db_files[0])
            conn = sqlite3.connect(db_path)
            cursor = conn.cursor()
            
            # Load messages for all topics
            self._load_all_messages_from_db(cursor)
            
            conn.close()
            
            # Check for and load existing annotations from the bag
            annotations_loaded = self.check_and_load_bag_annotations(bag_path)
            
            # Set up backward compatibility
            if self.video_topics:
                first_video_topic = list(self.video_topics.keys())[0]
                self.image_messages = self.video_topics[first_video_topic]
            
            # Calculate total duration
            if self.image_messages:
                start_time = self.image_messages[0][0]  # timestamp
                end_time = self.image_messages[-1][0]
                self.total_duration = (end_time - start_time) / 1e9  # Convert to seconds
            
            self.get_logger().info(f"Loaded bag with {len(self.available_topics)} topics")
            self.get_logger().info(f"Video topics: {list(self.video_topics.keys())}")
            if annotations_loaded:
                self.get_logger().info(f"Found and loaded {len(self.annotations)} existing annotations from bag")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to load bag file: {e}")
            return False
    
    def _parse_metadata(self, metadata):
        """Parse metadata to get topic information"""
        topics_info = metadata.get('rosbag2_bagfile_information', {}).get('topics_with_message_count', [])
        
        for topic_entry in topics_info:
            topic_metadata = topic_entry.get('topic_metadata', {})
            topic_name = topic_metadata.get('name', '')
            topic_type = topic_metadata.get('type', '')
            message_count = topic_entry.get('message_count', 0)
            
            self.available_topics[topic_name] = {
                'type': topic_type,
                'message_count': message_count,
                'is_video': self._is_video_topic(topic_type)
            }
    
    def _is_video_topic(self, topic_type: str) -> bool:
        """Check if a topic type represents video data"""
        video_types = [
            'sensor_msgs/msg/CompressedImage',
            'sensor_msgs/msg/Image'
        ]
        return topic_type in video_types
    
    def _load_all_messages_from_db(self, cursor):
        """Load messages from SQLite database for all topics"""
        try:
            # Get topic mapping from database
            cursor.execute("SELECT id, name, type FROM topics")
            topic_mapping = {name: (id, type) for id, name, type in cursor.fetchall()}
            
            # Load messages for each available topic
            for topic_name, topic_info in self.available_topics.items():
                if topic_name in topic_mapping:
                    topic_id, topic_type = topic_mapping[topic_name]
                    messages = self._load_topic_messages(cursor, topic_id, topic_type)
                    
                    if topic_info['is_video']:
                        self.video_topics[topic_name] = messages
                    else:
                        self.other_topics[topic_name] = messages
                        
                        # Legacy support for specific topics
                        if 'timestamp' in topic_name.lower():
                            self.timestamp_messages = messages
                        elif 'camera_info' in topic_name.lower():
                            self.camera_info_messages = messages
                        elif 'imu' in topic_name.lower():
                            self.imu_messages = messages
                            
        except Exception as e:
            self.get_logger().error(f"Error loading messages: {e}")
    
    def _load_topic_messages(self, cursor, topic_id: int, topic_type: str):
        """Load messages for a specific topic"""
        messages = []
        cursor.execute("""
            SELECT timestamp, data FROM messages 
            WHERE topic_id = ? ORDER BY timestamp
        """, (topic_id,))
        
        for timestamp, data in cursor.fetchall():
            try:
                # Deserialize the message
                msg_type = get_message(topic_type)
                msg = deserialize_message(data, msg_type)
                messages.append((timestamp, msg))
            except Exception as e:
                self.get_logger().warning(f"Failed to deserialize message for {topic_type}: {e}")
                continue
                
        return messages

    def check_and_load_bag_annotations(self, bag_path: str) -> bool:
        """Check if the bag contains annotations and load them"""
        try:
            # Check if /ev_annotations topic exists in available topics
            if '/ev_annotations' not in self.available_topics:
                return False
            
            # Find the database file
            db_files = [f for f in os.listdir(bag_path) if f.endswith('.db3')]
            if not db_files:
                return False
            
            db_path = os.path.join(bag_path, db_files[0])
            conn = sqlite3.connect(db_path)
            cursor = conn.cursor()
            
            # Get topic ID for annotations
            cursor.execute("SELECT id FROM topics WHERE name = ?", ('/ev_annotations',))
            result = cursor.fetchone()
            if not result:
                conn.close()
                return False
            
            topic_id = result[0]
            
            # Load annotation messages
            cursor.execute("""
                SELECT timestamp, data FROM messages 
                WHERE topic_id = ? ORDER BY timestamp
            """, (topic_id,))
            
            loaded_annotations = []
            for timestamp, data in cursor.fetchall():
                try:
                    # Deserialize the String message
                    string_msg_type = get_message('std_msgs/msg/String')
                    msg = deserialize_message(data, string_msg_type)
                    
                    # Parse the JSON annotation data
                    annotation_data = json.loads(msg.data)
                    
                    # Convert back to original annotation format
                    annotation = {
                        'annotation_id': annotation_data['id'],
                        'type': annotation_data['type'],
                        'text': annotation_data['text'],
                        'confidence': annotation_data['confidence'],
                        'timestamp': timestamp,
                        'datetime': annotation_data['datetime']
                    }
                    loaded_annotations.append(annotation)
                    
                except Exception as e:
                    self.get_logger().warning(f"Failed to parse annotation message: {e}")
                    continue
            
            conn.close()
            
            if loaded_annotations:
                self.annotations = loaded_annotations
                self.get_logger().info(f"Loaded {len(loaded_annotations)} annotations from bag file")
                return True
            
            return False
            
        except Exception as e:
            self.get_logger().error(f"Error checking for bag annotations: {e}")
            return False
