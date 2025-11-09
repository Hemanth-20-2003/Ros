#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    """ROS2 node for publishing camera images"""
    
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Publisher for camera images
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        
        # Camera setup
        # Change camera_id to 0 for USB camera, or use PiCamera if available
        self.camera_id = 0  # USB camera (change if needed)
        self.cap = cv2.VideoCapture(self.camera_id)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera {self.camera_id}')
            self.get_logger().error('Please check camera connection')
            return
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # Get actual camera properties
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        
        self.get_logger().info(f'Camera initialized: {width}x{height} @ {fps}FPS')
        
        # Timer for publishing frames
        self.publish_rate = 15  # FPS for publishing (lower than capture for efficiency)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_frame)
        
        self.frame_count = 0
        
        self.get_logger().info('Camera Publisher Node Started')
    
    def publish_frame(self):
        """Capture and publish a camera frame"""
        
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn('Failed to capture frame from camera')
            return
        
        try:
            # Convert OpenCV image to ROS message
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            
            # Add header information
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'camera_frame'
            
            # Publish the image
            self.publisher.publish(image_msg)
            
            self.frame_count += 1
            
            # Log every 100 frames
            if self.frame_count % 100 == 0:
                self.get_logger().info(f'Published {self.frame_count} camera frames')
                
        except Exception as e:
            self.get_logger().error(f'Error publishing frame: {e}')
    
    def destroy_node(self):
        """Clean up camera resources"""
        self.get_logger().info('Shutting down camera publisher...')
        
        if self.cap.isOpened():
            self.cap.release()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    camera_publisher = CameraPublisher()
    
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


