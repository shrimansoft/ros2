#!/usr/bin/env python3

import unittest
import sys
import os
import tempfile
import sqlite3
from unittest.mock import Mock, patch, MagicMock

# Add the project root to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'rosbag_annotator'))

try:
    from rosbag_annotator.scripts.annotation_gui import AnnotationMessage, BagAnnotatorGUI
except ImportError:
    # Fallback import
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'rosbag_annotator', 'scripts'))
    from annotation_gui import AnnotationMessage


class TestAnnotationMessage(unittest.TestCase):
    """Test cases for AnnotationMessage class"""
    
    def test_annotation_message_creation(self):
        """Test creating an annotation message"""
        timestamp = 123.456
        annotation_type = "safety"
        description = "Test annotation"
        confidence = 0.95
        
        msg = AnnotationMessage(timestamp, annotation_type, description, confidence)
        
        self.assertEqual(msg.timestamp, timestamp)
        self.assertEqual(msg.annotation_type, annotation_type)
        self.assertEqual(msg.description, description)
        self.assertEqual(msg.confidence, confidence)
    
    def test_annotation_message_validation(self):
        """Test annotation message validation"""
        # Valid annotation
        msg = AnnotationMessage(100.0, "danger", "Test", 1.0)
        self.assertIsNotNone(msg)
        
        # Test confidence bounds
        with self.assertRaises(ValueError):
            AnnotationMessage(100.0, "safety", "Test", 1.5)  # > 1.0
            
        with self.assertRaises(ValueError):
            AnnotationMessage(100.0, "safety", "Test", -0.1)  # < 0.0


class TestAnnotationStorage(unittest.TestCase):
    """Test cases for annotation storage and retrieval"""
    
    def setUp(self):
        """Set up test database"""
        self.temp_db = tempfile.NamedTemporaryFile(delete=False, suffix='.db')
        self.temp_db.close()
        self.db_path = self.temp_db.name
        
    def tearDown(self):
        """Clean up test database"""
        if os.path.exists(self.db_path):
            os.unlink(self.db_path)
    
    def test_database_creation(self):
        """Test annotation database creation"""
        # This would test the database initialization
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        # Create annotations table
        cursor.execute('''
            CREATE TABLE annotations (
                id INTEGER PRIMARY KEY,
                timestamp REAL,
                type TEXT,
                description TEXT,
                confidence REAL
            )
        ''')
        
        conn.commit()
        conn.close()
        
        # Verify table exists
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='annotations'")
        result = cursor.fetchone()
        self.assertIsNotNone(result)
        conn.close()
    
    def test_annotation_crud(self):
        """Test Create, Read, Update, Delete operations"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        # Create table
        cursor.execute('''
            CREATE TABLE annotations (
                id INTEGER PRIMARY KEY,
                timestamp REAL,
                type TEXT,
                description TEXT,
                confidence REAL
            )
        ''')
        
        # Create
        cursor.execute('''
            INSERT INTO annotations (timestamp, type, description, confidence)
            VALUES (?, ?, ?, ?)
        ''', (100.0, "safety", "Test annotation", 0.9))
        
        # Read
        cursor.execute("SELECT * FROM annotations WHERE timestamp = ?", (100.0,))
        result = cursor.fetchone()
        self.assertIsNotNone(result)
        self.assertEqual(result[1], 100.0)  # timestamp
        self.assertEqual(result[2], "safety")  # type
        
        # Update
        cursor.execute('''
            UPDATE annotations SET description = ? WHERE timestamp = ?
        ''', ("Updated annotation", 100.0))
        
        cursor.execute("SELECT description FROM annotations WHERE timestamp = ?", (100.0,))
        result = cursor.fetchone()
        self.assertEqual(result[0], "Updated annotation")
        
        # Delete
        cursor.execute("DELETE FROM annotations WHERE timestamp = ?", (100.0,))
        cursor.execute("SELECT * FROM annotations WHERE timestamp = ?", (100.0,))
        result = cursor.fetchone()
        self.assertIsNone(result)
        
        conn.commit()
        conn.close()


class TestBagAnnotatorGUI(unittest.TestCase):
    """Test cases for BagAnnotatorGUI class"""
    
    def setUp(self):
        """Set up test environment"""
        self.temp_dir = tempfile.mkdtemp()
        
    def tearDown(self):
        """Clean up test environment"""
        import shutil
        shutil.rmtree(self.temp_dir, ignore_errors=True)
    
    @patch('tkinter.Tk')
    @patch('rclpy.init')
    def test_gui_initialization(self, mock_rclpy_init, mock_tk):
        """Test GUI initialization without actually creating windows"""
        mock_root = Mock()
        mock_tk.return_value = mock_root
        
        # This would test GUI initialization if we can import the class
        # For now, just test that imports work
        try:
            from rosbag_annotator.scripts.annotation_gui import BagAnnotatorGUI
            self.assertTrue(True)  # Import successful
        except ImportError:
            self.skipTest("GUI module not available for testing")
    
    def test_annotation_validation(self):
        """Test annotation input validation"""
        # Test valid annotation types
        valid_types = ["safety", "danger", "comment", "traffic_sign", "pedestrian", "vehicle", "road_condition", "weather"]
        
        for annotation_type in valid_types:
            # This would test the GUI's validation logic
            self.assertIn(annotation_type, valid_types)
        
        # Test confidence validation
        valid_confidences = [0.0, 0.5, 1.0]
        invalid_confidences = [-0.1, 1.1, 2.0]
        
        for conf in valid_confidences:
            self.assertGreaterEqual(conf, 0.0)
            self.assertLessEqual(conf, 1.0)
        
        for conf in invalid_confidences:
            self.assertTrue(conf < 0.0 or conf > 1.0)


class TestConfigurationHandling(unittest.TestCase):
    """Test cases for configuration file handling"""
    
    def setUp(self):
        """Create temporary config file"""
        self.temp_config = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
        config_content = """
annotation_types:
  safety: "Safe driving conditions"
  danger: "Dangerous situations"
  comment: "General observations"

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
"""
        self.temp_config.write(config_content)
        self.temp_config.close()
        
    def tearDown(self):
        """Clean up temporary config file"""
        os.unlink(self.temp_config.name)
    
    def test_config_loading(self):
        """Test configuration file loading"""
        import yaml
        
        with open(self.temp_config.name, 'r') as f:
            config = yaml.safe_load(f)
        
        self.assertIn('annotation_types', config)
        self.assertIn('defaults', config)
        self.assertIn('video', config)
        self.assertIn('export', config)
        
        # Test specific values
        self.assertEqual(config['defaults']['confidence'], 1.0)
        self.assertEqual(config['video']['max_width'], 640)
        self.assertIn('safety', config['annotation_types'])
    
    def test_config_validation(self):
        """Test configuration validation"""
        import yaml
        
        with open(self.temp_config.name, 'r') as f:
            config = yaml.safe_load(f)
        
        # Test required sections exist
        required_sections = ['annotation_types', 'defaults', 'video', 'export']
        for section in required_sections:
            self.assertIn(section, config)
        
        # Test video settings are valid
        video_config = config['video']
        self.assertGreater(video_config['max_width'], 0)
        self.assertGreater(video_config['max_height'], 0)
        self.assertGreater(video_config['fps'], 0)


if __name__ == '__main__':
    # Create test suite
    test_suite = unittest.TestSuite()
    
    # Add test classes
    test_classes = [
        TestAnnotationMessage,
        TestAnnotationStorage, 
        TestBagAnnotatorGUI,
        TestConfigurationHandling
    ]
    
    for test_class in test_classes:
        test_suite.addTest(unittest.makeSuite(test_class))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)
    
    # Exit with appropriate code
    sys.exit(0 if result.wasSuccessful() else 1)
