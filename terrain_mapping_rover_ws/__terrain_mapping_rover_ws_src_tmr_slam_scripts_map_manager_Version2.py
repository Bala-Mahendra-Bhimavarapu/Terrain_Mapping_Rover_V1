#!/usr/bin/env python3
"""
Map Manager

Utility for managing saved maps - listing, loading, deleting.

Usage:
    ros2 run tmr_slam map_manager.py --list
    ros2 run tmr_slam map_manager.py --info map_name
    ros2 run tmr_slam map_manager. py --delete map_name
"""

import os
import sys
import argparse
from datetime import datetime
from pathlib import Path
import sqlite3
import shutil


class MapManager:
    """Manages RTAB-MAP database files."""
    
    def __init__(self, maps_dir: str = None):
        if maps_dir is None:
            maps_dir = os.path. expanduser(
                '~/terrain_mapping_rover_ws/src/tmr_slam/maps')
        self.maps_dir = Path(maps_dir)
    
    def list_maps(self):
        """List all available maps."""
        print("\n" + "="*60)
        print("  Available Maps")
        print("="*60 + "\n")
        
        if not self.maps_dir.exists():
            print("Maps directory does not exist.")
            return
        
        db_files = list(self. maps_dir.glob("*.db"))
        pgm_files = list(self. maps_dir.glob("*.pgm"))
        pcd_files = list(self.maps_dir.glob("*.pcd"))
        
        if not db_files and not pgm_files: 
            print("No maps found.")
            return
        
        # List database files
        if db_files:
            print("RTAB-MAP Databases (. db):")
            for db in sorted(db_files):
                size = db.stat().st_size / (1024 * 1024)  # MB
                mtime = datetime.fromtimestamp(db. stat().st_mtime)
                print(f"  - {db.stem}: {size:.1f} MB, {mtime. strftime('%Y-%m-%d %H:%M')}")
        
        # List 2D maps
        if pgm_files: 
            print("\n2D Maps (.pgm):")
            for pgm in sorted(pgm_files):
                print(f"  - {pgm.stem}")
        
        # List 3D maps
        if pcd_files:
            print("\n3D Point Clouds (.pcd):")
            for pcd in sorted(pcd_files):
                size = pcd.stat().st_size / (1024 * 1024)
                print(f"  - {pcd. stem}: {size:.1f} MB")
        
        print()
    
    def get_map_info(self, map_name: str):
        """Get detailed info about a map."""
        print(f"\n" + "="*60)
        print(f"  Map Info: {map_name}")
        print("="*60 + "\n")
        
        db_path = self. maps_dir / f"{map_name}.db"
        
        if not db_path.exists():
            print(f"Map '{map_name}' not found.")
            return
        
        # Basic file info
        size = db_path.stat().st_size / (1024 * 1024)
        mtime = datetime.fromtimestamp(db_path.stat().st_mtime)
        print(f"Database: {db_path}")
        print(f"Size:  {size:.2f} MB")
        print(f"Modified:  {mtime.strftime('%Y-%m-%d %H:%M:%S')}")
        
        # Try to read database info
        try:
            conn = sqlite3.connect(str(db_path))
            cursor = conn.cursor()
            
            # Count nodes
            cursor.execute("SELECT COUNT(*) FROM Node")
            node_count = cursor.fetchone()[0]
            print(f"Nodes: {node_count}")
            
            # Count links
            cursor.execute("SELECT COUNT(*) FROM Link")
            link_count = cursor.fetchone()[0]
            print(f"Links: {link_count}")
            
            # Count loop closures (link type 2)
            cursor.execute("SELECT COUNT(*) FROM Link WHERE type=2")
            loop_count = cursor.fetchone()[0]
            print(f"Loop Closures: {loop_count}")
            
            # Get map bounds if available
            try:
                cursor.execute("SELECT MIN(pose_x), MAX(pose_x), MIN(pose_y), MAX(pose_y) FROM Node")
                bounds = cursor.fetchone()
                if bounds[0] is not None: 
                    print(f"Map Bounds:")
                    print(f"  X: {bounds[0]:.2f} to {bounds[1]:.2f} m")
                    print(f"  Y: {bounds[2]:.2f} to {bounds[3]:.2f} m")
                    width = bounds[1] - bounds[0]
                    height = bounds[3] - bounds[2]
                    print(f"  Size: {width:.2f} x {height:.2f} m")
            except: 
                pass
            
            conn.close()
        except Exception as e:
            print(f"Could not read database details: {e}")
        
        # Check for associated files
        pgm_path = self.maps_dir / f"{map_name}. pgm"
        yaml_path = self.maps_dir / f"{map_name}.yaml"
        pcd_path = self.maps_dir / f"{map_name}. pcd"
        
        print("\nAssociated files:")
        print(f"  2D Map (. pgm): {'✅ Yes' if pgm_path.exists() else '❌ No'}")
        print(f"  Map YAML: {'✅ Yes' if yaml_path.exists() else '❌ No'}")
        print(f"  3D Cloud (. pcd): {'✅ Yes' if pcd_path.exists() else '❌ No'}")
        print()
    
    def delete_map(self, map_name: str, force: bool = False):
        """Delete a map and all associated files."""
        files_to_delete = [
            self.maps_dir / f"{map_name}.db",
            self.maps_dir / f"{map_name}.pgm",
            self.maps_dir / f"{map_name}.yaml",
            self.maps_dir / f"{map_name}.pcd",
            self.maps_dir / f"{map_name}. ply",
        ]
        
        existing = [f for f in files_to_delete if f.exists()]
        
        if not existing:
            print(f"Map '{map_name}' not found.")
            return
        
        print(f"\nFiles to delete:")
        for f in existing:
            print(f"  - {f.name}")
        
        if not force:
            confirm = input("\nDelete these files? (y/n): ")
            if confirm.lower() != 'y':
                print("Cancelled.")
                return
        
        for f in existing:
            try:
                f.unlink()
                print(f"Deleted: {f.name}")
            except Exception as e:
                print(f"Error deleting {f.name}: {e}")
        
        print(f"\nMap '{map_name}' deleted.")
    
    def copy_map(self, source_name: str, dest_name: str):
        """Copy a map to a new name."""
        source_db = self.maps_dir / f"{source_name}.db"
        
        if not source_db.exists():
            print(f"Source map '{source_name}' not found.")
            return
        
        dest_db = self.maps_dir / f"{dest_name}.db"
        if dest_db.exists():
            print(f"Destination map '{dest_name}' already exists.")
            return
        
        # Copy all associated files
        extensions = ['.db', '.pgm', '.yaml', '.pcd', '.ply']
        
        for ext in extensions:
            source = self.maps_dir / f"{source_name}{ext}"
            dest = self. maps_dir / f"{dest_name}{ext}"
            
            if source.exists():
                shutil.copy2(source, dest)
                print(f"Copied: {source.name} -> {dest.name}")
        
        print(f"\nMap copied:  {source_name} -> {dest_name}")
    
    def export_map(self, map_name: str, output_dir: str):
        """Export map files to a directory."""
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        extensions = ['.db', '.pgm', '.yaml', '.pcd', '.ply']
        exported = []
        
        for ext in extensions:
            source = self.maps_dir / f"{map_name}{ext}"
            if source.exists():
                dest = output_path / source.name
                shutil.copy2(source, dest)
                exported.append(source.name)
        
        if exported:
            print(f"Exported {len(exported)} files to {output_dir}")
            for f in exported:
                print(f"  - {f}")
        else:
            print(f"No files found for map '{map_name}'")


def main():
    parser = argparse.ArgumentParser(description='Map Manager for TMR SLAM')
    
    parser.add_argument('--list', '-l', action='store_true',
                       help='List all maps')
    parser.add_argument('--info', '-i', type=str, metavar='MAP_NAME',
                       help='Show info for a map')
    parser.add_argument('--delete', '-d', type=str, metavar='MAP_NAME',
                       help='Delete a map')
    parser.add_argument('--copy', '-c', nargs=2, metavar=('SRC', 'DEST'),
                       help='Copy a map')
    parser.add_argument('--export', '-e', nargs=2, metavar=('MAP_NAME', 'OUTPUT_DIR'),
                       help='Export map to directory')
    parser.add_argument('--force', '-f', action='store_true',
                       help='Skip confirmation prompts')
    parser.add_argument('--maps-dir', type=str,
                       help='Maps directory path')
    
    args = parser.parse_args()
    
    manager = MapManager(args.maps_dir)
    
    if args.list:
        manager.list_maps()
    elif args.info:
        manager.get_map_info(args.info)
    elif args.delete:
        manager.delete_map(args.delete, args.force)
    elif args.copy:
        manager.copy_map(args. copy[0], args.copy[1])
    elif args.export:
        manager.export_map(args.export[0], args.export[1])
    else:
        parser.print_help()


if __name__ == '__main__':
    main()