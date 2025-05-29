#!/usr/bin/env python3

import sys
import os
import json
import sqlite3
import cv2
import numpy as np
import yaml
import logging
import traceback
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Tuple
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CompressedImage, CameraInfo, Image as SensorImage
from std_msgs.msg import String, Header
from geometry_msgs.msg import Vector3Stamped
from cv_bridge import CvBridge

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from PIL import Image, ImageTk

def setup_logging():
    """Setup enhanced logging for debug mode"""
    # Check if debug mode is enabled via environment variable
    debug_mode = os.environ.get('ANNOTATOR_DEBUG', '0') == '1'
    log_dir = os.environ.get('ANNOTATOR_LOG_DIR', '/tmp')
    
    # Create log directory if it doesn't exist
    os.makedirs(log_dir, exist_ok=True)
    
    # Configure logging level
    log_level = logging.DEBUG if debug_mode else logging.INFO
    
    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s'
    )
    
    # Configure root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(log_level)
    
    # Remove existing handlers
    for handler in root_logger.handlers[:]:
        root_logger.removeHandler(handler)
    
    # Console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(log_level)
    console_handler.setFormatter(formatter)
    root_logger.addHandler(console_handler)
    
    # File handler for debug mode
    if debug_mode:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file = os.path.join(log_dir, f'annotator_debug_{timestamp}.log')
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(formatter)
        root_logger.addHandler(file_handler)
        
        # Error file handler
        error_file = os.path.join(log_dir, f'annotator_errors_{timestamp}.log')
        error_handler = logging.FileHandler(error_file)
        error_handler.setLevel(logging.ERROR)
        error_handler.setFormatter(formatter)
        root_logger.addHandler(error_handler)
        
        logging.info(f"Debug mode enabled. Logs will be saved to: {log_file}")
        logging.info(f"Error logs will be saved to: {error_file}")
    
    return debug_mode

def log_exception(exc_type, exc_value, exc_traceback):
    """Global exception handler for logging unhandled exceptions"""
    if issubclass(exc_type, KeyboardInterrupt):
        sys.__excepthook__(exc_type, exc_value, exc_traceback)
        return
    
    logging.error("Unhandled exception:", exc_info=(exc_type, exc_value, exc_traceback))
    logging.error("".join(traceback.format_exception(exc_type, exc_value, exc_traceback)))

# Set up logging and global exception handler
DEBUG_MODE = setup_logging()
sys.excepthook = log_exception

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
            logging.info(f"Starting to load bag file: {bag_path}")
            self.current_bag_path = bag_path
            
            # Validate bag path
            if not os.path.exists(bag_path):
                error_msg = f"Bag path does not exist: {bag_path}"
                logging.error(error_msg)
                return False
            
            if not os.path.isdir(bag_path):
                error_msg = f"Bag path is not a directory: {bag_path}"
                logging.error(error_msg)
                return False
            
            # Clear previous data
            self.available_topics.clear()
            self.video_topics.clear()
            self.other_topics.clear()
            self.image_messages.clear()
            logging.debug("Cleared previous data")
            
            # First, read metadata to get topic information
            metadata_path = os.path.join(bag_path, 'metadata.yaml')
            if os.path.exists(metadata_path):
                logging.debug(f"Loading metadata from: {metadata_path}")
                try:
                    with open(metadata_path, 'r') as f:
                        metadata = yaml.safe_load(f)
                    self._parse_metadata(metadata)
                    logging.info(f"Parsed metadata for {len(self.available_topics)} topics")
                except Exception as e:
                    logging.error(f"Failed to parse metadata: {e}")
                    return False
            else:
                logging.warning(f"No metadata.yaml found at: {metadata_path}")
            
            # Find the database file
            try:
                db_files = [f for f in os.listdir(bag_path) if f.endswith('.db3')]
                if not db_files:
                    error_msg = f"No database file found in bag directory: {bag_path}"
                    logging.error(error_msg)
                    self.get_logger().error(error_msg)
                    return False
                
                logging.info(f"Found database files: {db_files}")
                db_path = os.path.join(bag_path, db_files[0])
                logging.debug(f"Using database: {db_path}")
                
            except Exception as e:
                error_msg = f"Error accessing bag directory: {e}"
                logging.error(error_msg)
                return False
            
            # Connect to database and load messages
            try:
                conn = sqlite3.connect(db_path)
                cursor = conn.cursor()
                logging.debug("Connected to database successfully")
                
                # Load messages for all topics
                self._load_all_messages_from_db(cursor)
                conn.close()
                logging.info("Loaded all messages from database")
                
            except Exception as e:
                error_msg = f"Database operation failed: {e}"
                logging.error(error_msg, exc_info=True)
                if 'conn' in locals():
                    conn.close()
                return False
            
            # Set up backward compatibility
            if self.video_topics:
                first_video_topic = list(self.video_topics.keys())[0]
                self.image_messages = self.video_topics[first_video_topic]
                logging.debug(f"Set up backward compatibility with topic: {first_video_topic}")
            else:
                logging.warning("No video topics found for backward compatibility")
            
            # Calculate total duration
            if self.image_messages:
                start_time = self.image_messages[0][0]  # timestamp
                end_time = self.image_messages[-1][0]
                self.total_duration = (end_time - start_time) / 1e9  # Convert to seconds
            
            self.get_logger().info(f"Loaded bag with {len(self.available_topics)} topics")
            self.get_logger().info(f"Video topics: {list(self.video_topics.keys())}")
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
            logging.debug(f"Found {len(topic_mapping)} topics in database")
            
            # Load messages for each available topic
            for topic_name, topic_info in self.available_topics.items():
                if topic_name in topic_mapping:
                    topic_id, topic_type = topic_mapping[topic_name]
                    logging.debug(f"Loading messages for topic: {topic_name} (type: {topic_type})")
                    messages = self._load_topic_messages(cursor, topic_id, topic_type)
                    
                    if topic_info['is_video']:
                        self.video_topics[topic_name] = messages
                        logging.info(f"Loaded {len(messages)} video messages for {topic_name}")
                    else:
                        self.other_topics[topic_name] = messages
                        logging.info(f"Loaded {len(messages)} messages for {topic_name}")
                        
                        # Legacy support for specific topics
                        if 'timestamp' in topic_name.lower():
                            self.timestamp_messages = messages
                        elif 'camera_info' in topic_name.lower():
                            self.camera_info_messages = messages
                        elif 'imu' in topic_name.lower():
                            self.imu_messages = messages
                else:
                    logging.warning(f"Topic {topic_name} found in metadata but not in database")
                            
        except Exception as e:
            logging.error(f"Error loading messages: {e}", exc_info=True)
            self.get_logger().error(f"Error loading messages: {e}")
    
    def _load_topic_messages(self, cursor, topic_id: int, topic_type: str):
        """Load messages for a specific topic"""
        messages = []
        try:
            cursor.execute("""
                SELECT timestamp, data FROM messages 
                WHERE topic_id = ? ORDER BY timestamp
            """, (topic_id,))
            
            rows = cursor.fetchall()
            logging.debug(f"Found {len(rows)} messages for topic_id {topic_id}")
            
            for timestamp, data in rows:
                try:
                    # Deserialize the message
                    msg_type = get_message(topic_type)
                    msg = deserialize_message(data, msg_type)
                    messages.append((timestamp, msg))
                except Exception as e:
                    logging.warning(f"Failed to deserialize message for {topic_type}: {e}")
                    self.get_logger().warning(f"Failed to deserialize message for {topic_type}: {e}")
                    continue
                    
        except Exception as e:
            logging.error(f"Database query failed for topic_id {topic_id}: {e}", exc_info=True)
            
        return messages

    def get_frame_at_time(self, timestamp: int, topic_name: Optional[str] = None) -> Optional[np.ndarray]:
        """Get the image frame closest to the given timestamp from specified topic"""
        # Use first video topic if none specified
        if topic_name is None:
            if not self.video_topics:
                return None
            topic_name = list(self.video_topics.keys())[0]
        
        if topic_name not in self.video_topics:
            return None
            
        messages = self.video_topics[topic_name]
        if not messages:
            return None
        
        # Find closest message
        closest_idx = 0
        min_diff = abs(messages[0][0] - timestamp)
        
        for i, (msg_time, _) in enumerate(messages):
            diff = abs(msg_time - timestamp)
            if diff < min_diff:
                min_diff = diff
                closest_idx = i
        
        # Convert image to OpenCV format
        _, msg = messages[closest_idx]
        try:
            if hasattr(msg, 'format') and 'compressed' in str(type(msg)).lower():
                # CompressedImage
                cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            else:
                # Regular Image
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            return cv_image
        except Exception as e:
            self.get_logger().error(f"Error converting image from {topic_name}: {e}")
            return None
    
    def get_available_video_topics(self) -> List[str]:
        """Get list of available video topics"""
        return list(self.video_topics.keys())
    
    def get_all_topics_info(self) -> Dict:
        """Get information about all available topics"""
        return self.available_topics
    
    def add_annotation(self, annotation_type: str, text: str, confidence: float = 1.0):
        """Add an annotation at the current time"""
        if self.current_frame_time is None:
            return
        
        annotation = {
            'timestamp': self.current_frame_time,
            'annotation_id': f"ann_{len(self.annotations):04d}",
            'type': annotation_type,
            'text': text,
            'confidence': confidence,
            'datetime': datetime.fromtimestamp(self.current_frame_time / 1e9).isoformat()
        }
        
        self.annotations.append(annotation)
        self.get_logger().info(f"Added annotation: {annotation}")
    
    def save_annotations(self, output_path: str):
        """Save annotations to JSON file"""
        try:
            with open(output_path, 'w') as f:
                json.dump(self.annotations, f, indent=2)
            self.get_logger().info(f"Saved {len(self.annotations)} annotations to {output_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save annotations: {e}")
    
    def load_annotations(self, input_path: str):
        """Load annotations from JSON file"""
        try:
            with open(input_path, 'r') as f:
                self.annotations = json.load(f)
            self.get_logger().info(f"Loaded {len(self.annotations)} annotations from {input_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load annotations: {e}")

class AnnotationGUI:
    def __init__(self, annotator: RosbagAnnotator):
        self.annotator = annotator
        self.root = tk.Tk()
        self.root.title("ROS2 Bag Annotator - Dynamic Multi-Topic Video Player")
        self.root.geometry("1600x1000")
        
        # Current frame
        self.current_frame = None
        self.current_frame_idx = 0
        
        # Multiple video displays
        self.video_displays = {}  # topic_name -> video_label
        self.selected_topics = []  # Topics selected for display
        
        # Playback thread
        self.playback_thread = None
        self.playback_running = False
        
        self.setup_ui()
        
    def setup_ui(self):
        """Setup the user interface"""
        # Main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Menu bar
        self.setup_menu()
        
        # Control panel
        control_frame = ttk.LabelFrame(main_frame, text="Playback Controls")
        control_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Load bag button
        ttk.Button(control_frame, text="Load Bag File", 
                  command=self.load_bag_file).pack(side=tk.LEFT, padx=5)
        
        # Topic selection button
        self.topic_select_button = ttk.Button(control_frame, text="Select Video Topics", 
                                            command=self.show_topic_selection, state=tk.DISABLED)
        self.topic_select_button.pack(side=tk.LEFT, padx=5)
        
        # Show all topics button
        self.show_topics_button = ttk.Button(control_frame, text="Show All Topics", 
                                           command=self.show_all_topics, state=tk.DISABLED)
        self.show_topics_button.pack(side=tk.LEFT, padx=5)
        
        # Playback controls
        self.play_button = ttk.Button(control_frame, text="Play", 
                                     command=self.toggle_playback)
        self.play_button.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(control_frame, text="Previous", 
                  command=self.previous_frame).pack(side=tk.LEFT, padx=2)
        ttk.Button(control_frame, text="Next", 
                  command=self.next_frame).pack(side=tk.LEFT, padx=2)
        
        # Speed control
        ttk.Label(control_frame, text="Speed:").pack(side=tk.LEFT, padx=(10, 2))
        self.speed_var = tk.DoubleVar(value=1.0)
        speed_scale = ttk.Scale(control_frame, from_=0.1, to=3.0, 
                               variable=self.speed_var, orient=tk.HORIZONTAL, length=100)
        speed_scale.pack(side=tk.LEFT, padx=2)
        
        # Progress bar
        progress_frame = ttk.Frame(main_frame)
        progress_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.progress_var = tk.DoubleVar()
        self.progress_bar = ttk.Scale(progress_frame, from_=0, to=100, 
                                     variable=self.progress_var, orient=tk.HORIZONTAL)
        self.progress_bar.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)
        self.progress_bar.bind("<Button-1>", self.seek_to_position)
        
        self.time_label = ttk.Label(progress_frame, text="00:00 / 00:00")
        self.time_label.pack(side=tk.RIGHT)
        
        # Main content area with paned window
        paned_window = ttk.PanedWindow(main_frame, orient=tk.HORIZONTAL)
        paned_window.pack(fill=tk.BOTH, expand=True)
        
        # Left side: Video displays
        video_frame = ttk.LabelFrame(paned_window, text="Video Displays")
        paned_window.add(video_frame, weight=3)
        
        # Notebook for multiple video feeds
        self.video_notebook = ttk.Notebook(video_frame)
        self.video_notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Default video display (for when no topics are selected)
        self.default_video_frame = ttk.Frame(self.video_notebook)
        self.video_notebook.add(self.default_video_frame, text="No Video Loaded")
        
        self.video_label = ttk.Label(self.default_video_frame, text="Load a bag file and select video topics")
        self.video_label.pack(expand=True)
        
        # Right side: Topic info and annotation panel
        right_frame = ttk.Frame(paned_window)
        paned_window.add(right_frame, weight=1)
        
        # Topic information panel
        self.setup_topic_info_panel(right_frame)
        
        # Annotation panel
        self.setup_annotation_panel(right_frame)
    
    def setup_topic_info_panel(self, parent):
        """Setup the topic information panel"""
        topic_frame = ttk.LabelFrame(parent, text="Topic Information")
        topic_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        # Treeview for topic information
        self.topics_tree = ttk.Treeview(topic_frame, columns=('Type', 'Count'), show='headings', height=8)
        self.topics_tree.heading('#0', text='Topic Name')
        self.topics_tree.heading('#1', text='Type')
        self.topics_tree.heading('#2', text='Message Count')
        self.topics_tree.column('#0', width=200)
        self.topics_tree.column('#1', width=180)
        self.topics_tree.column('#2', width=100)
        
        # Scrollbar for topics tree
        topic_scrollbar = ttk.Scrollbar(topic_frame, orient=tk.VERTICAL, command=self.topics_tree.yview)
        topic_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.topics_tree.configure(yscrollcommand=topic_scrollbar.set)
        self.topics_tree.pack(fill=tk.BOTH, expand=True)
        
        # Show video topic type
        self.topics_tree['show'] = 'tree headings'
    
    def setup_annotation_panel(self, parent):
        """Setup the annotation panel"""
        annotation_frame = ttk.LabelFrame(parent, text="Annotations")
        annotation_frame.pack(fill=tk.BOTH, expand=True)
        
        # Annotation type selection
        type_frame = ttk.Frame(annotation_frame)
        type_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(type_frame, text="Annotation Type:").pack(anchor=tk.W)
        self.annotation_type = tk.StringVar(value="safety")
        types = ["safety", "danger", "comment", "traffic_sign", "pedestrian", "vehicle"]
        
        # Create a sub-frame for radio buttons to arrange them in columns
        radio_frame = ttk.Frame(type_frame)
        radio_frame.pack(fill=tk.X, pady=5)
        
        for i, ann_type in enumerate(types):
            row = i // 2
            col = i % 2
            ttk.Radiobutton(radio_frame, text=ann_type.title(), 
                           variable=self.annotation_type, 
                           value=ann_type).grid(row=row, column=col, sticky=tk.W, padx=5)
        
        # Annotation text
        text_frame = ttk.Frame(annotation_frame)
        text_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(text_frame, text="Annotation Text:").pack(anchor=tk.W)
        self.annotation_text = tk.Text(text_frame, height=4, width=30)
        self.annotation_text.pack(fill=tk.X)
        
        # Confidence slider
        conf_frame = ttk.Frame(annotation_frame)
        conf_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(conf_frame, text="Confidence:").pack(anchor=tk.W)
        self.confidence_var = tk.DoubleVar(value=1.0)
        conf_scale = ttk.Scale(conf_frame, from_=0.0, to=1.0, 
                              variable=self.confidence_var, orient=tk.HORIZONTAL)
        conf_scale.pack(fill=tk.X)
        
        # Add annotation button
        ttk.Button(annotation_frame, text="Add Annotation", 
                  command=self.add_annotation).pack(pady=10)
        
        # Annotations list
        list_frame = ttk.Frame(annotation_frame)
        list_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        ttk.Label(list_frame, text="Current Annotations:").pack(anchor=tk.W)
        
        # Treeview for annotations
        self.annotations_tree = ttk.Treeview(list_frame, columns=('Type', 'Text'), show='headings', height=6)
        self.annotations_tree.heading('#1', text='Type')
        self.annotations_tree.heading('#2', text='Text')
        self.annotations_tree.column('#1', width=80)
        self.annotations_tree.column('#2', width=150)
        self.annotations_tree.pack(fill=tk.BOTH, expand=True)
        
        # Scrollbar for annotations tree
        ann_scrollbar = ttk.Scrollbar(list_frame, orient=tk.VERTICAL, command=self.annotations_tree.yview)
        ann_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.annotations_tree.configure(yscrollcommand=ann_scrollbar.set)
        
        # Delete annotation button
        ttk.Button(annotation_frame, text="Delete Selected", 
                  command=self.delete_annotation).pack(pady=5)
    
    def setup_menu(self):
        """Setup menu bar"""
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)
        
        # File menu
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Load Bag File", command=self.load_bag_file)
        file_menu.add_separator()
        file_menu.add_command(label="Save Annotations", command=self.save_annotations)
        file_menu.add_command(label="Load Annotations", command=self.load_annotations_file)
        file_menu.add_separator()
        file_menu.add_command(label="Export Annotated Bag", command=self.export_annotated_bag)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.quit)
        
        # View menu
        view_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="View", menu=view_menu)
        view_menu.add_command(label="Show All Topics", command=self.show_all_topics)
        view_menu.add_command(label="Select Video Topics", command=self.show_topic_selection)
    
    def load_bag_file(self):
        """Load a ROS2 bag file"""
        try:
            logging.info("User initiated bag file loading")
            
            # Default to /data directory (mounted from host)
            initial_dir = "/data" if os.path.exists("/data") else os.path.expanduser("~")
            logging.debug(f"Using initial directory: {initial_dir}")
            
            # Show info about available directories
            info_msg = "File Access Information:\n"
            info_msg += "• /data - Main data directory (mounted from host)\n"
            if os.path.exists("/home_data"):
                info_msg += "• /home_data - Your home directory (read-only)\n"
            info_msg += "\nSelect the rosbag directory containing .db3 files"
            
            messagebox.showinfo("File Browser", info_msg)
            
            bag_path = filedialog.askdirectory(
                title="Select ROS2 Bag Directory", 
                initialdir=initial_dir
            )
            
            if not bag_path:
                logging.info("User cancelled bag file selection")
                return
                
            logging.info(f"User selected bag path: {bag_path}")
            
            # Attempt to load the bag file
            if self.annotator.load_bag_file(bag_path):
                logging.info("Bag file loaded successfully")
                self.current_frame_idx = 0
                self.update_topic_display()
                self.update_frame_display()
                self.update_annotations_list()
                
                # Enable topic selection buttons
                self.topic_select_button.config(state=tk.NORMAL)
                self.show_topics_button.config(state=tk.NORMAL)
                
                success_msg = (f"Loaded bag file: {bag_path}\n"
                             f"Topics found: {len(self.annotator.available_topics)}\n"
                             f"Video topics: {len(self.annotator.video_topics)}")
                logging.info(success_msg)
                messagebox.showinfo("Success", success_msg)
            else:
                error_msg = f"Failed to load bag file: {bag_path}"
                logging.error(error_msg)
                messagebox.showerror("Error", f"Failed to load the bag file.\n\nPath: {bag_path}\n\nCheck the logs for more details.")
                
        except Exception as e:
            error_msg = f"Unexpected error during bag file loading: {e}"
            logging.error(error_msg, exc_info=True)
            messagebox.showerror("Error", f"An unexpected error occurred:\n\n{str(e)}\n\nCheck the logs for more details.")
    
    def show_all_topics(self):
        """Show information about all topics in the bag"""
        if not self.annotator.available_topics:
            messagebox.showwarning("Warning", "No bag file loaded")
            return
        
        # Create a new window to display all topics
        topics_window = tk.Toplevel(self.root)
        topics_window.title("All Topics in Bag File")
        topics_window.geometry("800x600")
        
        # Create treeview for detailed topic information
        tree_frame = ttk.Frame(topics_window)
        tree_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        tree = ttk.Treeview(tree_frame, columns=('Type', 'Messages', 'Video'), show='tree headings')
        tree.heading('#0', text='Topic Name')
        tree.heading('#1', text='Message Type')
        tree.heading('#2', text='Message Count')
        tree.heading('#3', text='Video Topic')
        
        tree.column('#0', width=250)
        tree.column('#1', width=250)
        tree.column('#2', width=100)
        tree.column('#3', width=80)
        
        # Add scrollbar
        scrollbar = ttk.Scrollbar(tree_frame, orient=tk.VERTICAL, command=tree.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        tree.configure(yscrollcommand=scrollbar.set)
        tree.pack(fill=tk.BOTH, expand=True)
        
        # Populate with topic information
        for topic_name, topic_info in self.annotator.available_topics.items():
            is_video = "Yes" if topic_info['is_video'] else "No"
            tree.insert("", tk.END, text=topic_name, values=(
                topic_info['type'],
                topic_info['message_count'],
                is_video
            ))
        
        # Add close button
        ttk.Button(topics_window, text="Close", command=topics_window.destroy).pack(pady=10)
    
    def show_topic_selection(self):
        """Show dialog to select video topics for display"""
        if not self.annotator.video_topics:
            messagebox.showwarning("Warning", "No video topics found in bag file")
            return
        
        # Create topic selection window
        selection_window = tk.Toplevel(self.root)
        selection_window.title("Select Video Topics to Display")
        selection_window.geometry("600x400")
        
        ttk.Label(selection_window, text="Select video topics to display simultaneously:").pack(pady=10)
        
        # Create checkboxes for video topics
        self.topic_vars = {}
        topic_frame = ttk.Frame(selection_window)
        topic_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)
        
        for topic_name in self.annotator.video_topics.keys():
            var = tk.BooleanVar()
            self.topic_vars[topic_name] = var
            ttk.Checkbutton(topic_frame, text=topic_name, variable=var).pack(anchor=tk.W, pady=2)
        
        # Buttons
        button_frame = ttk.Frame(selection_window)
        button_frame.pack(pady=10)
        
        ttk.Button(button_frame, text="Apply Selection", 
                  command=lambda: self.apply_topic_selection(selection_window)).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Cancel", 
                  command=selection_window.destroy).pack(side=tk.LEFT, padx=5)
    
    def apply_topic_selection(self, selection_window):
        """Apply the selected video topics for display"""
        self.selected_topics = [topic for topic, var in self.topic_vars.items() if var.get()]
        
        if not self.selected_topics:
            messagebox.showwarning("Warning", "Please select at least one video topic")
            return
        
        # Clear existing video displays
        for i in range(self.video_notebook.index("end")):
            self.video_notebook.forget(0)
        
        self.video_displays.clear()
        
        # Create new video displays for selected topics
        for topic_name in self.selected_topics:
            frame = ttk.Frame(self.video_notebook)
            self.video_notebook.add(frame, text=topic_name.split('/')[-1])  # Use topic name as tab title
            
            label = ttk.Label(frame, text=f"Loading {topic_name}...")
            label.pack(expand=True)
            self.video_displays[topic_name] = label
        
        selection_window.destroy()
        self.update_frame_display()
        
        messagebox.showinfo("Success", f"Selected {len(self.selected_topics)} video topics for display")
    
    def update_topic_display(self):
        """Update the topic information display"""
        # Clear existing items
        for item in self.topics_tree.get_children():
            self.topics_tree.delete(item)
        
        # Add topic information
        for topic_name, topic_info in self.annotator.available_topics.items():
            item_id = self.topics_tree.insert("", tk.END, text=topic_name, values=(
                topic_info['type'],
                topic_info['message_count']
            ))
            
            # Highlight video topics
            if topic_info['is_video']:
                self.topics_tree.set(item_id, 'Type', f"🎥 {topic_info['type']}")
    
    def toggle_playback(self):
        """Toggle video playback"""
        if not self.annotator.image_messages:
            messagebox.showwarning("Warning", "No bag file loaded")
            return
        
        if self.playback_running:
            self.stop_playback()
        else:
            self.start_playback()
    
    def start_playback(self):
        """Start video playback"""
        self.playback_running = True
        self.play_button.config(text="Pause")
        self.playback_thread = threading.Thread(target=self.playback_loop)
        self.playback_thread.daemon = True
        self.playback_thread.start()
    
    def stop_playback(self):
        """Stop video playback"""
        self.playback_running = False
        self.play_button.config(text="Play")
    
    def playback_loop(self):
        """Playback loop running in separate thread"""
        while self.playback_running and self.current_frame_idx < len(self.annotator.image_messages):
            self.root.after(0, self.update_frame_display)
            self.current_frame_idx += 1
            
            # Calculate delay based on speed
            delay = (1.0 / 30.0) / self.speed_var.get()  # Assuming 30 FPS
            time.sleep(delay)
        
        self.root.after(0, lambda: self.play_button.config(text="Play"))
        self.playback_running = False
    
    def previous_frame(self):
        """Go to previous frame"""
        if self.current_frame_idx > 0:
            self.current_frame_idx -= 1
            self.update_frame_display()
    
    def next_frame(self):
        """Go to next frame"""
        if self.current_frame_idx < len(self.annotator.image_messages) - 1:
            self.current_frame_idx += 1
            self.update_frame_display()
    
    def seek_to_position(self, event):
        """Seek to a specific position in the video"""
        if not self.annotator.image_messages:
            return
        
        # Calculate new frame index based on progress bar position
        progress = self.progress_var.get()
        new_idx = int((progress / 100.0) * len(self.annotator.image_messages))
        self.current_frame_idx = max(0, min(new_idx, len(self.annotator.image_messages) - 1))
        self.update_frame_display()
    
    def update_frame_display(self):
        """Update the video frame display for all selected topics"""
        if not self.annotator.image_messages or self.current_frame_idx >= len(self.annotator.image_messages):
            return
        
        # Get current timestamp
        timestamp, _ = self.annotator.image_messages[self.current_frame_idx]
        self.annotator.current_frame_time = timestamp
        
        # Update each selected video display
        if self.selected_topics:
            for topic_name in self.selected_topics:
                if topic_name in self.video_displays:
                    frame = self.annotator.get_frame_at_time(timestamp, topic_name)
                    if frame is not None:
                        self.display_frame_in_label(frame, self.video_displays[topic_name])
        else:
            # Default behavior for single video display
            frame = self.annotator.get_frame_at_time(timestamp)
            if frame is not None:
                self.display_frame_in_label(frame, self.video_label)
        
        # Update progress bar
        if len(self.annotator.image_messages) > 0:
            progress = (self.current_frame_idx / len(self.annotator.image_messages)) * 100
            self.progress_var.set(progress)
        
        # Update time display
        current_time = self.current_frame_idx / 30.0  # Assuming 30 FPS
        total_time = len(self.annotator.image_messages) / 30.0
        time_str = f"{self.format_time(current_time)} / {self.format_time(total_time)}"
        self.time_label.config(text=time_str)
    
    def display_frame_in_label(self, frame: np.ndarray, label: ttk.Label):
        """Display a frame in the specified label"""
        # Resize frame for display
        height, width = frame.shape[:2]
        max_width, max_height = 400, 300
        
        if width > max_width or height > max_height:
            scale = min(max_width / width, max_height / height)
            new_width = int(width * scale)
            new_height = int(height * scale)
            frame = cv2.resize(frame, (new_width, new_height))
        
        # Convert to RGB and create PhotoImage
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(frame_rgb)
        photo = ImageTk.PhotoImage(image)
        
        label.configure(image=photo, text="")
        # Store reference in the AnnotationGUI class to prevent garbage collection
        if not hasattr(self, '_image_references'):
            self._image_references = []
        self._image_references.append(photo)
    
    def format_time(self, seconds):
        """Format time in MM:SS format"""
        minutes = int(seconds // 60)
        seconds = int(seconds % 60)
        return f"{minutes:02d}:{seconds:02d}"
    
    def add_annotation(self):
        """Add a new annotation"""
        text = self.annotation_text.get("1.0", tk.END).strip()
        if not text:
            messagebox.showwarning("Warning", "Please enter annotation text")
            return
        
        ann_type = self.annotation_type.get()
        confidence = self.confidence_var.get()
        
        self.annotator.add_annotation(ann_type, text, confidence)
        self.annotation_text.delete("1.0", tk.END)
        self.update_annotations_list()
    
    def delete_annotation(self):
        """Delete selected annotation"""
        selection = self.annotations_tree.selection()
        if not selection:
            messagebox.showwarning("Warning", "Please select an annotation to delete")
            return
        
        # Get the index of the selected item
        item = selection[0]
        idx = self.annotations_tree.index(item)
        
        if 0 <= idx < len(self.annotator.annotations):
            del self.annotator.annotations[idx]
            self.update_annotations_list()
    
    def update_annotations_list(self):
        """Update the annotations list display"""
        # Clear existing items
        for item in self.annotations_tree.get_children():
            self.annotations_tree.delete(item)
        
        # Add current annotations
        for ann in self.annotator.annotations:
            text_preview = ann['text'][:30] + "..." if len(ann['text']) > 30 else ann['text']
            self.annotations_tree.insert("", tk.END, values=(ann['type'], text_preview))
    
    def save_annotations(self):
        """Save annotations to file"""
        if not self.annotator.annotations:
            messagebox.showwarning("Warning", "No annotations to save")
            return
        
        # Default to /data directory (mounted from host)
        initial_dir = "/data" if os.path.exists("/data") else os.path.expanduser("~")
        file_path = filedialog.asksaveasfilename(
            initialdir=initial_dir,
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if file_path:
            self.annotator.save_annotations(file_path)
            messagebox.showinfo("Success", f"Annotations saved to {file_path}")
    
    def load_annotations_file(self):
        """Load annotations from file"""
        # Default to /data directory (mounted from host)
        initial_dir = "/data" if os.path.exists("/data") else os.path.expanduser("~")
        file_path = filedialog.askopenfilename(
            initialdir=initial_dir,
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if file_path:
            self.annotator.load_annotations(file_path)
            self.update_annotations_list()
            messagebox.showinfo("Success", f"Annotations loaded from {file_path}")
    
    def _ask_export_name(self):
        """Ask user for a custom export name"""
        from tkinter import simpledialog
        
        # Generate default name based on timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_name = f"annotated_bag_{timestamp}"
        
        # Prompt user for custom name
        name = simpledialog.askstring(
            "Export Name",
            "Enter a name for the exported bag file:",
            initialvalue=default_name
        )
        
        if name:
            # Clean the name to be filesystem-safe
            import re
            name = re.sub(r'[<>:"/\\|?*]', '_', name)
            name = name.strip()
            
        return name

    def export_annotated_bag(self):
        """Export a new bag file with annotations"""
        if not self.annotator.annotations:
            messagebox.showwarning("Warning", "No annotations to export")
            return
        
        if not self.annotator.current_bag_path:
            messagebox.showwarning("Warning", "No bag file loaded")
            return
        
        # Ask for custom export name
        export_name = self._ask_export_name()
        if not export_name:
            return  # User cancelled or didn't provide a name
        
        # Default to /data directory (mounted from host)
        initial_dir = "/data" if os.path.exists("/data") else os.path.expanduser("~")
        output_dir = filedialog.askdirectory(
            title="Select Output Directory",
            initialdir=initial_dir
        )
        if output_dir:
            self._perform_export(output_dir, export_name)
    
    def _perform_export(self, output_dir, export_name):
        """Perform the actual export process"""
        import tempfile
        import subprocess
        import threading
        
        try:
            # Create temporary annotations file
            temp_annotations_file = tempfile.mktemp(suffix='.json')
            self.annotator.save_annotations(temp_annotations_file)
            
            # Use the custom export name
            output_bag_path = os.path.join(output_dir, export_name)
            
            # Show progress dialog
            progress_dialog = tk.Toplevel(self.root)
            progress_dialog.title("Exporting Annotated Bag")
            progress_dialog.geometry("400x150")
            progress_dialog.transient(self.root)
            progress_dialog.grab_set()
            
            ttk.Label(progress_dialog, text="Creating annotated bag file...").pack(pady=20)
            
            progress_bar = ttk.Progressbar(progress_dialog, mode='indeterminate')
            progress_bar.pack(pady=10, padx=20, fill=tk.X)
            progress_bar.start()
            
            status_label = ttk.Label(progress_dialog, text="Initializing...")
            status_label.pack(pady=5)
            
            # Function to run the merger in a separate thread
            def run_merger():
                try:
                    status_label.config(text="Running annotation merger...")
                    
                    # Run the annotation merger script
                    cmd = [
                        'python3', 
                        '-m', 'rosbag_annotator.scripts.annotation_merger',
                        self.annotator.current_bag_path,
                        temp_annotations_file,
                        output_bag_path
                    ]
                    
                    result = subprocess.run(cmd, capture_output=True, text=True, cwd='/ros2_ws/src/rosbag_annotator')
                    
                    # Clean up temporary file
                    os.unlink(temp_annotations_file)
                    
                    # Update UI on main thread
                    def update_ui():
                        progress_bar.stop()
                        progress_dialog.destroy()
                        
                        if result.returncode == 0:
                            messagebox.showinfo(
                                "Export Successful", 
                                f"Annotated bag created successfully!\n\nOutput: {output_bag_path}\n\nThe bag now contains:\n"
                                f"• All original topics and data\n"
                                f"• New '/ev_annotations' topic with {len(self.annotator.annotations)} annotations\n"
                                f"• Support for multi-topic video display"
                            )
                        else:
                            error_msg = f"Export failed!\n\nError: {result.stderr}\n\nStdout: {result.stdout}"
                            messagebox.showerror("Export Failed", error_msg)
                    
                    self.root.after(0, update_ui)
                    
                except Exception as e:
                    # Clean up temporary file
                    if os.path.exists(temp_annotations_file):
                        os.unlink(temp_annotations_file)
                    
                    # Update UI on main thread
                    def update_ui():
                        progress_bar.stop()
                        progress_dialog.destroy()
                        messagebox.showerror("Export Error", f"An error occurred during export:\n\n{str(e)}")
                    
                    self.root.after(0, update_ui)
            
            # Start merger in background thread
            merger_thread = threading.Thread(target=run_merger)
            merger_thread.daemon = True
            merger_thread.start()
            
        except Exception as e:
            messagebox.showerror("Export Error", f"Failed to start export process:\n\n{str(e)}")

    def run(self):
        """Run the GUI application"""
        self.root.mainloop()

def main():
    """Main entry point for the enhanced annotation GUI"""
    logging.info("Starting Enhanced ROS2 Bag Annotator GUI")
    
    if DEBUG_MODE:
        logging.debug("Debug mode is enabled - detailed logging active")
    
    rclpy.init()
    
    try:
        logging.info("Initializing ROS2 annotator node")
        annotator = RosbagAnnotator()
        
        logging.info("Creating GUI application")
        gui = AnnotationGUI(annotator)
        
        logging.info("Starting GUI main loop")
        gui.run()
        
    except KeyboardInterrupt:
        logging.info("Application interrupted by user")
    except Exception as e:
        logging.error(f"Unhandled exception in main: {e}", exc_info=True)
        if DEBUG_MODE:
            import traceback
            traceback.print_exc()
        raise
    finally:
        logging.info("Shutting down ROS2")
        rclpy.shutdown()

if __name__ == '__main__':
    main()
