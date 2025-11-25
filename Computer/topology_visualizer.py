# topology_visualizer.py - Server-side Topology Visualization Module
# This module creates graphical representations of the environment using bot sensor data
# Provides real-time visualization of scanned areas and bot navigation

import matplotlib.pyplot as plt
import numpy as np
import math
from collections import deque
import time
from typing import List, Tuple, Deque

class TopologyVisualizer:
    """
    Topology Visualization System for Mobile Bot Environment Mapping
    Converts polar sensor data to Cartesian coordinates and creates real-time maps
    Auto-scales display and connects points to represent environment outlines
    """
    
    def __init__(self, max_points = 200, update_interval = 1000):
        """
        Initialize the topology visualizer with display parameters
        
        Args:
            max_points: Maximum number of points to store and display (manages memory)
            update_interval: Refresh interval for auto-update in milliseconds
        """
        # Data storage configuration
        self.max_points = max_points
        self.update_interval = update_interval
        
        # Data storage structures
        self.all_points = deque(maxlen=max_points)  # All (x, y) coordinates
        self.scan_sequences = []  # Complete scan sequences for connection
        
        # Initialize matplotlib figure and axes for plotting
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        
        # Setup the plot with proper formatting and labels
        self.setup_plot()
        
        # Auto-scaling parameters for dynamic display adjustment
        self.x_min, self.x_max = 0, 0
        self.y_min, self.y_max = 0, 0
        self.auto_scaling = True  # Enable automatic plot scaling
        
    def setup_plot(self):
        """
        Initialize the plot with proper formatting, labels, and styling
        Creates a professional-looking visualization environment
        """
        # Set plot title with formatting
        self.ax.set_title('Mobile Bot Environment Topology', 
                         fontsize=16, fontweight='bold', pad=20)
        
        # Set axis labels
        self.ax.set_xlabel('X Coordinate (cm)', fontsize=12)
        self.ax.set_ylabel('Y Coordinate (cm)', fontsize=12)
        
        # Configure grid for better readability
        self.ax.grid(True, alpha=0.3, linestyle='--')
        
        # Add center reference lines
        self.ax.axhline(y=0, color='black', linestyle='-', alpha=0.5, linewidth=0.5)
        self.ax.axvline(x=0, color='black', linestyle='-', alpha=0.5, linewidth=0.5)
        
        # Set equal aspect ratio for proper scaling
        self.ax.set_aspect('equal', adjustable='datalim')
        
        # Initialize plot elements that will be updated
        self.scatter_plot = self.ax.scatter([], [], c='blue', s=30, alpha=0.7, 
                                          label='Obstacle Points')
        self.line_plots = []  # Store line objects for each scan sequence
        self.bot_position = self.ax.scatter([0], [0], c='red', s=100, marker='o', 
                                          label='Bot Position', edgecolors='black')
        
        # Add legend to identify plot elements
        self.ax.legend(loc='upper right', fontsize=10)
        
    def polar_to_cartesian(self, angle_degrees, distance_cm):
        """
        Convert polar coordinates (angle, distance) to Cartesian coordinates (x, y)
        Adjusts for mathematical convention where 0° = East, 90° = North
        
        Args:
            angle_degrees: Angle in degrees (0-360) from bot's forward direction
            distance_cm: Distance measurement in centimeters
            
        Returns:
            tuple: (x, y) Cartesian coordinates in centimeters
        """
        # Convert angle to radians and adjust for mathematical coordinate system
        # 90° adjustment makes 0° = North, 90° = East (standard navigation convention)
        angle_radians = math.radians(90 - angle_degrees)
        
        # Calculate Cartesian coordinates using trigonometry
        x = distance_cm * math.cos(angle_radians)
        y = distance_cm * math.sin(angle_radians)
        
        return x, y
        
    def add_sensor_data(self, angle_degrees, distance_cm):
        """
        Add individual sensor data point to the topology visualization
        Filters invalid measurements and updates auto-scaling bounds
        
        Args:
            angle_degrees: Sensor angle in degrees (0-360)
            distance_cm: Measured distance in centimeters
        """
        # Filter out invalid distance measurements
        if distance_cm <= 0 or distance_cm > 1000:
            return
            
        # Convert polar coordinates to Cartesian coordinates
        x, y = self.polar_to_cartesian(angle_degrees, distance_cm)
        
        # Add point to the main points collection
        self.all_points.append((x, y))
        
        # Update auto-scaling boundaries to include new point
        self._update_scaling_bounds(x, y)
        
    def add_scan_data(self, scan_data):
        """
        Add a complete scan sequence to the topology visualization
        Stores scan sequences separately for point connection and outline creation
        
        Args:
            scan_data: List of (angle, distance) tuples from a single 180-degree scan
        """
        valid_points = []
        
        # Process each data point in the scan
        for angle, distance in scan_data:
            # Validate distance measurements
            if distance > 0 and distance <= 1000:  # Reasonable distance range
                # Convert to Cartesian coordinates
                x, y = self.polar_to_cartesian(angle, distance)
                valid_points.append((x, y))
                
                # Add to main points collection
                self.all_points.append((x, y))
                
                # Update scaling bounds
                self._update_scaling_bounds(x, y)
        
        # Store this scan sequence for connecting points (environment outlines)
        if valid_points:
            self.scan_sequences.append(valid_points)
            
        # Manage memory by keeping only recent scans (last 10 scans)
        if len(self.scan_sequences) > 10:
            self.scan_sequences.pop(0)
            
    def _update_scaling_bounds(self, x, y):
        """
        Update the auto-scaling boundaries based on new data points
        Ensures all points remain visible in the plot
        
        Args:
            x: X coordinate of new point
            y: Y coordinate of new point
        """
        if len(self.all_points) == 1:
            # First point - set initial bounds with generous margin
            self.x_min, self.x_max = x - 50, x + 50
            self.y_min, self.y_max = y - 50, y + 50
        else:
            # Expand bounds if new point is outside current range
            margin = 20  # 20cm margin around points for better visibility
            
            self.x_min = min(self.x_min, x - margin)
            self.x_max = max(self.x_max, x + margin)
            self.y_min = min(self.y_min, y - margin)
            self.y_max = max(self.y_max, y + margin)
            
    def auto_scale_plot(self):
        """
        Automatically adjust plot limits based on current data points
        Adds margins and ensures minimum display range for small datasets
        """
        if not self.all_points:
            # No data available - set default view
            self.ax.set_xlim(-100, 100)
            self.ax.set_ylim(-100, 100)
            return
            
        # Calculate current data range
        x_range = self.x_max - self.x_min
        y_range = self.y_max - self.y_min
        
        # Add 10% margin to the bounds for better visibility
        x_margin = x_range * 0.1 if x_range > 0 else 50
        y_margin = y_range * 0.1 if y_range > 0 else 50
        
        # Ensure minimum range for small datasets
        if x_range < 100:
            x_center = (self.x_min + self.x_max) / 2
            self.ax.set_xlim(x_center - 50, x_center + 50)
        else:
            self.ax.set_xlim(self.x_min - x_margin, self.x_max + x_margin)
            
        if y_range < 100:
            y_center = (self.y_min + self.y_max) / 2
            self.ax.set_ylim(y_center - 50, y_center + 50)
        else:
            self.ax.set_ylim(self.y_min - y_margin, self.y_max + y_margin)
        
    def connect_scan_points(self, scan_points):
        """
        Connect points from a single scan to represent environment outline
        Sorts points by angle and creates closed polygons for better visualization
        
        Args:
            scan_points: List of (x, y) points from one complete scan
            
        Returns:
            tuple: (x_coords, y_coords) for connected line plot
        """
        if len(scan_points) < 2:
            return [], []  # Need at least 2 points to create a line
            
        # Sort points by angle (approximated by atan2) for proper connection order
        # This creates a continuous outline around the environment
        sorted_points = sorted(scan_points, 
                             key=lambda p: math.atan2(p[1], p[0]))
        
        # Extract x and y coordinates from sorted points
        x_coords = [p[0] for p in sorted_points]
        y_coords = [p[1] for p in sorted_points]
        
        # Close the polygon by connecting last point to first point
        # Only if we have enough points to form a meaningful shape
        if len(sorted_points) > 2:
            x_coords.append(x_coords[0])
            y_coords.append(y_coords[0])
            
        return x_coords, y_coords
        
    def update_display(self):
        """
        Update the graphical display with current data
        Refreshes scatter plots, line connections, and auto-scaling
        """
        # Clear existing line plots to prevent duplication
        for line in self.line_plots:
            line.remove()
        self.line_plots.clear()
        
        # Check if we have data to display
        if not self.all_points:
            return
            
        # Update scatter plot with all accumulated points
        x_all = [p[0] for p in self.all_points]
        y_all = [p[1] for p in self.all_points]
        self.scatter_plot.set_offsets(np.column_stack([x_all, y_all]))
        
        # Create line plots for each stored scan sequence
        for scan_points in self.scan_sequences:
            if len(scan_points) >= 2:
                # Connect points to form environment outlines
                x_line, y_line = self.connect_scan_points(scan_points)
                if x_line and y_line:
                    # Create line plot for this scan sequence
                    line_plot, = self.ax.plot(x_line, y_line, 'g-', 
                                            alpha=0.6, linewidth=1.5,
                                            label='Environment Outline' if not self.line_plots else "")
                    self.line_plots.append(line_plot)
        
        # Apply auto-scaling if enabled
        if self.auto_scaling:
            self.auto_scale_plot()
            
        # Refresh the plot canvas to show updates
        self.fig.canvas.draw_idle()
        
    def show_plot(self):
        """
        Display the plot in a blocking call
        Opens the matplotlib window and keeps it active
        """
        self.update_display()
        plt.show()
        
    def save_topology_image(self, filename = "topology_map.png"):
        """
        Save the current topology map as a high-resolution image file
        Useful for documentation, reporting, and analysis
        
        Args:
            filename: Output filename for the saved image
        """
        # Ensure display is updated before saving
        self.update_display()
        
        # Save figure with high quality settings
        self.fig.savefig(filename, dpi=300, bbox_inches='tight', 
                        facecolor='white', edgecolor='none')
        print(f"Topology map saved as {filename}")
        
    def clear_data(self):
        """
        Clear all stored data and reset the plot
        Useful for starting new mapping sessions or debugging
        """
        # Clear all data storage
        self.all_points.clear()
        self.scan_sequences.clear()
        
        # Remove all line plots from the display
        for line in self.line_plots:
            line.remove()
        self.line_plots.clear()
        
        # Reset scaling bounds
        self.x_min, self.x_max = 0, 0
        self.y_min, self.y_max = 0, 0
        
        # Refresh the display
        self.update_display()
        
    def get_statistics(self):
        """
        Get statistics about the current topology data
        Provides insights into mapping progress and coverage
        
        Returns:
            dict: Statistics including point count, area coverage, etc.
        """
        stats = {
            'total_points': len(self.all_points),
            'total_scans': len(self.scan_sequences),
            'x_range': (self.x_min, self.x_max),
            'y_range': (self.y_min, self.y_max),
            'coverage_area': (self.x_max - self.x_min) * (self.y_max - self.y_min)
        }
        return stats

# Example usage and demonstration
def main():
    """
    Demonstrate the TopologyVisualizer with sample data
    Shows basic functionality and typical usage patterns
    """
    
    # Create visualizer instance with custom parameters
    visualizer = TopologyVisualizer(max_points=100, update_interval=500)
    
    # Generate sample scan data simulating a rectangular room
    sample_scans = [
        # First scan - rectangular room pattern
        [(0, 100), (45, 141), (90, 100), (135, 141), (180, 100), 
         (225, 141), (270, 100), (315, 141)],
         
        # Second scan - slightly different positions (simulating movement)
        [(0, 95), (45, 136), (90, 95), (135, 136), (180, 95),
         (225, 136), (270, 95), (315, 136)]
    ]
    
    print("Adding sample scan data to topology visualizer...")
    
    # Add sample scans to the visualizer
    for i, scan_data in enumerate(sample_scans):
        print(f"Adding scan {i+1} with {len(scan_data)} points")
        visualizer.add_scan_data(scan_data)
        
    # Update the display to show the data
    visualizer.update_display()
    
    # Display statistics about the collected data
    stats = visualizer.get_statistics()
    print(f"\nTopology Statistics:")
    print(f"Total points: {stats['total_points']}")
    print(f"Total scans: {stats['total_scans']}")
    print(f"X Range: {stats['x_range'][0]:.1f} to {stats['x_range'][1]:.1f} cm")
    print(f"Y Range: {stats['y_range'][0]:.1f} to {stats['y_range'][1]:.1f} cm")
    print(f"Coverage Area: {stats['coverage_area']:.1f} cm²")
    
    # Save the topology map as an image file
    visualizer.save_topology_image("sample_topology.png")
    
    # Display the interactive plot window
    print("\nDisplaying topology map... Close the window to exit.")
    visualizer.show_plot()

# Conditional execution for testing
if __name__ == "__main__":
    main()