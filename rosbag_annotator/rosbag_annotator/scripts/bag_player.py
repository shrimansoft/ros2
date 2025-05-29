#!/usr/bin/env python3

import sys
import os
import sqlite3
import json
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

class BagPlayer(Node):
    def __init__(self):
        super().__init__('bag_player')
        
        # Publishers
        self.image_pub = self.create_publisher(CompressedImage, '/image_raw/compressed', 10)
        self.annotation_pub = self.create_publisher(String, '/ev_annotations', 10)
        
        # Timer for playback
        self.timer = None
        self.messages = []
        self.current_idx = 0
        self.playback_rate = 1.0
        
    def load_bag(self, bag_path: str):
        """Load messages from bag file"""
        try:
            db_path = None
            
            # Find the database file
            for file in os.listdir(bag_path):
                if file.endswith('.db3'):
                    db_path = os.path.join(bag_path, file)
                    break
            
            if not db_path:
                self.get_logger().error("No database file found in bag directory")
                return False
            
            conn = sqlite3.connect(db_path)
            cursor = conn.cursor()
            
            # Load all messages with timestamps
            cursor.execute("""
                SELECT m.timestamp, t.name, t.type, m.data
                FROM messages m
                JOIN topics t ON m.topic_id = t.id
                ORDER BY m.timestamp
            """)
            
            for timestamp, topic_name, topic_type, data in cursor.fetchall():
                self.messages.append({
                    'timestamp': timestamp,
                    'topic': topic_name,
                    'type': topic_type,
                    'data': data
                })
            
            conn.close()
            
            self.get_logger().info(f"Loaded {len(self.messages)} messages from bag")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to load bag: {e}")
            return False
    
    def start_playback(self, rate: float = 1.0):
        """Start playing back the bag"""
        if not self.messages:
            self.get_logger().error("No messages loaded")
            return
        
        self.playback_rate = rate
        self.current_idx = 0
        
        # Calculate playback interval (assuming 30 Hz for images)
        interval = (1.0 / 30.0) / self.playback_rate
        
        self.timer = self.create_timer(interval, self.playback_callback)
        self.get_logger().info(f"Started playback at {rate}x speed")
    
    def stop_playback(self):
        """Stop playback"""
        if self.timer:
            self.timer.cancel()
            self.timer = None
        self.get_logger().info("Stopped playback")
    
    def playback_callback(self):
        """Callback for message playback"""
        if self.current_idx >= len(self.messages):
            self.stop_playback()
            self.get_logger().info("Playback finished")
            return
        
        message_info = self.messages[self.current_idx]
        
        try:
            if message_info['topic'] == '/image_raw/compressed':
                # Deserialize and publish image
                msg_type = get_message('sensor_msgs/msg/CompressedImage')
                msg = deserialize_message(message_info['data'], msg_type)
                self.image_pub.publish(msg)
                
            elif message_info['topic'] == '/ev_annotations':
                # Deserialize and publish annotation
                msg_type = get_message('std_msgs/msg/String')
                msg = deserialize_message(message_info['data'], msg_type)
                self.annotation_pub.publish(msg)
                
                # Parse and log annotation
                try:
                    annotation = json.loads(msg.data)
                    self.get_logger().info(f"Annotation: {annotation['type']} - {annotation['text']}")
                except:
                    self.get_logger().info(f"Annotation: {msg.data}")
        
        except Exception as e:
            self.get_logger().error(f"Error publishing message: {e}")
        
        self.current_idx += 1
    
    def seek_to_time(self, target_time: int):
        """Seek to a specific timestamp"""
        for i, msg in enumerate(self.messages):
            if msg['timestamp'] >= target_time:
                self.current_idx = i
                self.get_logger().info(f"Seeked to message {i}")
                break

def main():
    rclpy.init()
    
    try:
        player = BagPlayer()
        
        if len(sys.argv) < 2:
            print("Usage: bag_player.py <bag_directory> [playback_rate]")
            sys.exit(1)
        
        bag_path = sys.argv[1]
        playback_rate = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
        
        if player.load_bag(bag_path):
            player.start_playback(playback_rate)
            
            print(f"Playing bag from {bag_path}")
            print("Press Ctrl+C to stop")
            
            rclpy.spin(player)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
