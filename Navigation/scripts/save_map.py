#!/usr/bin/env python3
"""
Map saver utility for SLAM.
Saves maps created by SLAM Toolbox or Cartographer to disk.
"""

import os
import sys
import argparse
from datetime import datetime
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import yaml
import numpy as np
from PIL import Image


class MapSaver(Node):
    def __init__(self, map_name, map_dir):
        super().__init__('map_saver')
        self.map_name = map_name
        self.map_dir = map_dir
        self.map_received = False
        
        # Create output directory if it doesn't exist
        os.makedirs(self.map_dir, exist_ok=True)
        
        # QoS profile for map subscription
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # Subscribe to map topic
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile
        )
        
        self.get_logger().info(f'Waiting for map on topic /map...')
    
    def map_callback(self, msg):
        if self.map_received:
            return
            
        self.get_logger().info('Map received! Saving...')
        self.map_received = True
        
        # Save map image (PGM format)
        map_image_file = os.path.join(self.map_dir, f'{self.map_name}.pgm')
        self.save_map_image(msg, map_image_file)
        
        # Save map metadata (YAML format)
        map_yaml_file = os.path.join(self.map_dir, f'{self.map_name}.yaml')
        self.save_map_yaml(msg, map_yaml_file, f'{self.map_name}.pgm')
        
        self.get_logger().info(f'Map saved to {self.map_dir}')
        self.get_logger().info(f'  Image: {self.map_name}.pgm')
        self.get_logger().info(f'  Metadata: {self.map_name}.yaml')
        
        # Shutdown after saving
        rclpy.shutdown()
    
    def save_map_image(self, occupancy_grid, filename):
        """Save occupancy grid as PGM image."""
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        
        # Convert occupancy grid data to image
        # OccupancyGrid: -1 (unknown), 0 (free), 100 (occupied)
        # Image: 205 (unknown/gray), 254 (free/white), 0 (occupied/black)
        data = np.array(occupancy_grid.data, dtype=np.int8).reshape((height, width))
        
        # Convert to image format
        img_data = np.zeros((height, width), dtype=np.uint8)
        img_data[data == -1] = 205  # Unknown - gray
        img_data[data == 0] = 254   # Free - white
        img_data[data == 100] = 0   # Occupied - black
        
        # Flip vertically (ROS uses different coordinate system)
        img_data = np.flipud(img_data)
        
        # Save as PGM
        img = Image.fromarray(img_data, mode='L')
        img.save(filename)
    
    def save_map_yaml(self, occupancy_grid, yaml_file, image_file):
        """Save map metadata as YAML."""
        metadata = {
            'image': image_file,
            'resolution': float(occupancy_grid.info.resolution),
            'origin': [
                float(occupancy_grid.info.origin.position.x),
                float(occupancy_grid.info.origin.position.y),
                0.0
            ],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
        
        with open(yaml_file, 'w') as f:
            yaml.dump(metadata, f, default_flow_style=False)


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Save SLAM map to disk',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Save map with default name (timestamp)
  ros2 run somanet save_map.py
  
  # Save map with custom name
  ros2 run somanet save_map.py --name my_warehouse_map
  
  # Save to custom directory
  ros2 run somanet save_map.py --dir ~/maps
        """
    )
    
    parser.add_argument(
        '--name',
        type=str,
        default=datetime.now().strftime('map_%Y%m%d_%H%M%S'),
        help='Map name (default: map_YYYYMMDD_HHMMSS)'
    )
    
    parser.add_argument(
        '--dir',
        type=str,
        default='./maps',
        help='Output directory (default: ./maps)'
    )
    
    parsed_args = parser.parse_args()
    
    rclpy.init(args=args)
    
    map_saver = MapSaver(parsed_args.name, parsed_args.dir)
    
    try:
        rclpy.spin(map_saver)
    except KeyboardInterrupt:
        pass
    finally:
        map_saver.destroy_node()


if __name__ == '__main__':
    main()
