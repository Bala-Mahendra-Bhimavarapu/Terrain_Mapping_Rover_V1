#!/usr/bin/env python3
"""
Costmap Diagnostics

Monitors costmap health and terrain layer performance.

Usage:
    ros2 run tmr_costmap costmap_diagnostics.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import os
import time
from collections import deque


class CostmapDiagnosticsNode(Node):
    """Monitors costmap performance."""
    
    def __init__(self):
        super().__init__('costmap_diagnostics')
        
        # Statistics
        self.local_costmap_times = deque(maxlen=100)
        self.global_costmap_times = deque(maxlen=100)
        self.point_cloud_times = deque(maxlen=100)
        
        self.local_costmap_size = (0, 0)
        self.global_costmap_size = (0, 0)
        self.point_cloud_count = 0
        
        self.local_costmap_stats = {'free': 0, 'occupied': 0, 'unknown': 0}
        
        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )
        
        # Subscribers
        self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap',
            self.local_costmap_callback, 10)
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap',
            self.global_costmap_callback, 10)
        self.create_subscription(
            PointCloud2, '/tof/points',
            self.point_cloud_callback, sensor_qos)
        
        # Publisher
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        
        # Timers
        self.diag_timer = self.create_timer(1.0, self.publish_diagnostics)
        self.display_timer = self.create_timer(0.5, self.update_display)
        
        self.get_logger().info("Costmap Diagnostics started")
    
    def local_costmap_callback(self, msg: OccupancyGrid):
        self.local_costmap_times.append(time.time())
        self.local_costmap_size = (msg.info.width, msg.info.height)
        
        # Compute statistics
        free = 0
        occupied = 0
        unknown = 0
        
        for cost in msg.data:
            if cost == 0:
                free += 1
            elif cost == 255 or cost == -1:
                unknown += 1
            else:
                occupied += 1
        
        self.local_costmap_stats = {
            'free': free,
            'occupied': occupied,
            'unknown': unknown
        }
    
    def global_costmap_callback(self, msg: OccupancyGrid):
        self.global_costmap_times.append(time.time())
        self.global_costmap_size = (msg.info.width, msg.info.height)
    
    def point_cloud_callback(self, msg: PointCloud2):
        self.point_cloud_times.append(time.time())
        self.point_cloud_count = msg.width * msg.height
    
    def calculate_rate(self, times: deque) -> float:
        if len(times) < 2:
            return 0.0
        duration = times[-1] - times[0]
        if duration <= 0:
            return 0.0
        return (len(times) - 1) / duration
    
    def publish_diagnostics(self):
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Costmap status
        status = DiagnosticStatus()
        status.name = "Costmap: Terrain Layer"
        status.hardware_id = "tmr_costmap"
        
        local_rate = self.calculate_rate(self.local_costmap_times)
        global_rate = self.calculate_rate(self.global_costmap_times)
        pc_rate = self.calculate_rate(self.point_cloud_times)
        
        status.values.append(KeyValue(key='Local Costmap Rate', value=f"{local_rate:.1f} Hz"))
        status.values.append(KeyValue(key='Global Costmap Rate', value=f"{global_rate:.1f} Hz"))
        status.values.append(KeyValue(key='Local Costmap Size', 
            value=f"{self.local_costmap_size[0]}x{self.local_costmap_size[1]}"))
        status.values.append(KeyValue(key='Point Cloud Rate', value=f"{pc_rate:.1f} Hz"))
        status.values.append(KeyValue(key='Point Count', value=str(self.point_cloud_count)))
        status.values.append(KeyValue(key='Free Cells', value=str(self.local_costmap_stats['free'])))
        status.values.append(KeyValue(key='Occupied Cells', value=str(self.local_costmap_stats['occupied'])))
        
        if local_rate > 1.0 and pc_rate > 5.0:
            status.level = DiagnosticStatus.OK
            status.message = "Costmap updating normally"
        elif local_rate > 0:
            status.level = DiagnosticStatus.WARN
            status.message = "Low update rate"
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = "Costmap not updating"
        
        diag_msg.status.append(status)
        self.diag_pub.publish(diag_msg)
    
    def update_display(self):
        os.system('clear' if os.name == 'posix' else 'cls')
        
        local_rate = self.calculate_rate(self.local_costmap_times)
        global_rate = self.calculate_rate(self.global_costmap_times)
        pc_rate = self.calculate_rate(self.point_cloud_times)
        
        print("╔" + "═"*58 + "╗")
        print("║" + "  COSTMAP DIAGNOSTICS".center(58) + "║")
        print("╠" + "═"*58 + "╣")
        
        # Input status
        print("║ POINT CLOUD INPUT:".ljust(59) + "║")
        pc_status = "✅" if pc_rate > 5 else "❌"
        print(f"║   {pc_status} /tof/points: {pc_rate:.1f} Hz, {self.point_cloud_count} points".ljust(59) + "║")
        
        print("╠" + "═"*58 + "╣")
        
        # Local costmap
        print("║ LOCAL COSTMAP:".ljust(59) + "║")
        local_status = "✅" if local_rate > 1 else "❌"
        print(f"║   {local_status} Update Rate: {local_rate:.1f} Hz".ljust(59) + "║")
        print(f"║   Size: {self.local_costmap_size[0]} x {self.local_costmap_size[1]} cells".ljust(59) + "║")
        
        total = sum(self.local_costmap_stats.values())
        if total > 0:
            free_pct = 100 * self.local_costmap_stats['free'] / total
            occ_pct = 100 * self.local_costmap_stats['occupied'] / total
            unk_pct = 100 * self.local_costmap_stats['unknown'] / total
            print(f"║   Free: {free_pct:.1f}%  Occupied: {occ_pct:.1f}%  Unknown: {unk_pct:.1f}%".ljust(59) + "║")
        
        print("╠" + "═"*58 + "╣")
        
        # Global costmap
        print("║ GLOBAL COSTMAP:".ljust(59) + "║")
        global_status = "✅" if global_rate > 0.5 else "⚠️"
        print(f"║   {global_status} Update Rate: {global_rate:.1f} Hz".ljust(59) + "║")
        print(f"║   Size: {self.global_costmap_size[0]} x {self.global_costmap_size[1]} cells".ljust(59) + "║")
        
        print("╠" + "═"*58 + "╣")
        
        # Overall status
        all_ok = local_rate > 1 and pc_rate > 5
        if all_ok:
            print("║  ✅ TERRAIN LAYER OPERATIONAL".ljust(59) + "║")
        else:
            print("║  ⚠️  TERRAIN LAYER DEGRADED".ljust(59) + "║")
        
        print("╠" + "═"*58 + "╣")
        print("║  Press Ctrl+C to exit".ljust(59) + "║")
        print("╚" + "═"*58 + "╝")


def main():
    rclpy.init()
    node = CostmapDiagnosticsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
