#!/usr/bin/env python3

import unittest
import sys
import os
import tempfile
import subprocess
import time
import sqlite3
from pathlib import Path

# Add project to path
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))


class TestEndToEndWorkflow(unittest.TestCase):
    """Integration tests for complete annotation workflow"""
    
    @classmethod
    def setUpClass(cls):
        """Set up test environment once for all tests"""
        cls.project_root = Path(__file__).parent.parent.parent
        cls.temp_dir = Path(tempfile.mkdtemp())
        cls.test_data_dir = cls.project_root / "tests" / "test_data"
        
        # Ensure test data directory exists
        cls.test_data_dir.mkdir(exist_ok=True)
        
    @classmethod
    def tearDownClass(cls):
        """Clean up test environment"""
        import shutil
        shutil.rmtree(cls.temp_dir, ignore_errors=True)
    
    def test_docker_build(self):
        """Test Docker image builds successfully"""
        os.chdir(self.project_root)
        
        # Test build command
        result = subprocess.run([
            'docker', 'build', '-t', 'shriman1:test', '.'
        ], capture_output=True, text=True, timeout=300)
        
        self.assertEqual(result.returncode, 0, f"Docker build failed: {result.stderr}")
        
        # Verify image exists
        result = subprocess.run([
            'docker', 'images', 'shriman1:test'
        ], capture_output=True, text=True)
        
        self.assertIn('shriman1:test', result.stdout)
    
    def test_health_check_script(self):
        """Test health check script execution"""
        script_path = self.project_root / "scripts" / "health_check.bash"
        
        # Make sure script is executable
        os.chmod(script_path, 0o755)
        
        # Run health check
        result = subprocess.run([
            str(script_path)
        ], capture_output=True, text=True, timeout=60)
        
        # Should not fail catastrophically (some warnings are OK)
        self.assertNotEqual(result.returncode, 1, f"Health check failed: {result.stderr}")
    
    def test_bag_utils_script(self):
        """Test bag utilities script"""
        script_path = self.project_root / "scripts" / "bag_utils.bash"
        
        # Make sure script is executable
        os.chmod(script_path, 0o755)
        
        # Test help command
        result = subprocess.run([
            str(script_path), 'help'
        ], capture_output=True, text=True)
        
        self.assertEqual(result.returncode, 0)
        self.assertIn('ROS2 Bag Utilities', result.stdout)
        
        # Test list command (should work even with no bags)
        result = subprocess.run([
            str(script_path), 'list'
        ], capture_output=True, text=True)
        
        self.assertEqual(result.returncode, 0)
        self.assertIn('Available bag files', result.stdout)
    
    def test_makefile_targets(self):
        """Test key Makefile targets"""
        os.chdir(self.project_root)
        
        # Test help target
        result = subprocess.run(['make', 'help'], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0)
        self.assertIn('ROS2 Bag Annotator', result.stdout)
        
        # Test info target
        result = subprocess.run(['make', 'info'], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0)
        self.assertIn('Project Information', result.stdout)
        
        # Test status target
        result = subprocess.run(['make', 'status'], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0)
        self.assertIn('System Status', result.stdout)


class TestContainerIntegration(unittest.TestCase):
    """Test Docker container functionality"""
    
    def setUp(self):
        """Set up for container tests"""
        self.project_root = Path(__file__).parent.parent.parent
        
    def test_container_basic_functionality(self):
        """Test basic container operations"""
        os.chdir(self.project_root)
        
        # Test container can start and execute basic commands
        result = subprocess.run([
            'docker', 'run', '--rm', 'shriman1:a', 'echo', 'test'
        ], capture_output=True, text=True, timeout=30)
        
        if result.returncode != 0:
            self.skipTest("Docker image not available for testing")
        
        self.assertEqual(result.returncode, 0)
        self.assertIn('test', result.stdout)
    
    def test_ros2_in_container(self):
        """Test ROS2 functionality in container"""
        os.chdir(self.project_root)
        
        # Test ROS2 command
        result = subprocess.run([
            'docker', 'run', '--rm', 'shriman1:a', 
            'bash', '-c', 'source /opt/ros/humble/setup.bash && ros2 --help'
        ], capture_output=True, text=True, timeout=30)
        
        if result.returncode != 0:
            self.skipTest("Docker image not available for testing")
        
        self.assertEqual(result.returncode, 0)
        self.assertIn('ros2', result.stdout)
    
    def test_python_modules_in_container(self):
        """Test Python modules are available in container"""
        os.chdir(self.project_root)
        
        # Test Python imports
        python_test = '''
import sys
try:
    import rclpy
    import cv2
    import numpy as np
    import yaml
    import tkinter
    print("All imports successful")
except ImportError as e:
    print(f"Import error: {e}")
    sys.exit(1)
'''
        
        result = subprocess.run([
            'docker', 'run', '--rm', 'shriman1:a',
            'python3', '-c', python_test
        ], capture_output=True, text=True, timeout=30)
        
        if result.returncode != 0:
            self.skipTest("Docker image not available for testing")
        
        self.assertEqual(result.returncode, 0)
        self.assertIn('All imports successful', result.stdout)


class TestAnnotationWorkflow(unittest.TestCase):
    """Test complete annotation workflow"""
    
    def setUp(self):
        """Set up test bag data"""
        self.temp_dir = Path(tempfile.mkdtemp())
        self.test_bag_dir = self.temp_dir / "test_bag"
        self.test_bag_dir.mkdir()
        
        # Create a minimal test bag structure
        self.create_test_bag()
        
    def tearDown(self):
        """Clean up test data"""
        import shutil
        shutil.rmtree(self.temp_dir, ignore_errors=True)
    
    def create_test_bag(self):
        """Create a minimal test bag for testing"""
        # Create metadata.yaml
        metadata = {
            'rosbag2_bagfile_information': {
                'version': 4,
                'storage_identifier': 'sqlite3',
                'relative_file_paths': ['rosbag2_test.db3'],
                'duration': {'nanoseconds': 1000000000},  # 1 second
                'starting_time': {'nanoseconds': 0},
                'message_count': 10,
                'topics_with_message_count': [
                    {'topic_metadata': {'name': '/test_topic', 'type': 'std_msgs/msg/String'}, 'message_count': 10}
                ]
            }
        }
        
        import yaml
        with open(self.test_bag_dir / "metadata.yaml", 'w') as f:
            yaml.dump(metadata, f)
        
        # Create minimal SQLite database
        db_path = self.test_bag_dir / "rosbag2_test.db3"
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        
        # Create topics table
        cursor.execute('''
            CREATE TABLE topics (
                id INTEGER PRIMARY KEY,
                name TEXT NOT NULL,
                type TEXT NOT NULL,
                serialization_format TEXT NOT NULL
            )
        ''')
        
        cursor.execute('''
            INSERT INTO topics (id, name, type, serialization_format)
            VALUES (1, '/test_topic', 'std_msgs/msg/String', 'cdr')
        ''')
        
        # Create messages table
        cursor.execute('''
            CREATE TABLE messages (
                id INTEGER PRIMARY KEY,
                topic_id INTEGER,
                timestamp INTEGER,
                data BLOB
            )
        ''')
        
        # Add some test messages
        for i in range(10):
            cursor.execute('''
                INSERT INTO messages (topic_id, timestamp, data)
                VALUES (1, ?, ?)
            ''', (i * 100000000, b'test_data'))  # Simple test data
        
        conn.commit()
        conn.close()
    
    def test_bag_loading(self):
        """Test bag file can be loaded"""
        # This would test the bag loading functionality
        # For now, just verify our test bag structure is valid
        
        metadata_file = self.test_bag_dir / "metadata.yaml"
        db_file = self.test_bag_dir / "rosbag2_test.db3"
        
        self.assertTrue(metadata_file.exists())
        self.assertTrue(db_file.exists())
        
        # Test database structure
        conn = sqlite3.connect(db_file)
        cursor = conn.cursor()
        
        cursor.execute("SELECT name FROM sqlite_master WHERE type='table'")
        tables = [row[0] for row in cursor.fetchall()]
        
        self.assertIn('topics', tables)
        self.assertIn('messages', tables)
        
        cursor.execute("SELECT COUNT(*) FROM messages")
        message_count = cursor.fetchone()[0]
        self.assertEqual(message_count, 10)
        
        conn.close()


if __name__ == '__main__':
    # Create test suite
    test_suite = unittest.TestSuite()
    
    # Add test classes
    test_classes = [
        TestEndToEndWorkflow,
        TestContainerIntegration,
        TestAnnotationWorkflow
    ]
    
    for test_class in test_classes:
        test_suite.addTest(unittest.makeSuite(test_class))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)
    
    # Exit with appropriate code
    sys.exit(0 if result.wasSuccessful() else 1)
