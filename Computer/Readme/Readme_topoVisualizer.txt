# Topology Visualizer

**File**: `topology_visualizer.py`  
**Location**: Server Side  
**Purpose**: Real-time environment mapping and visualization

## 🎯 Overview

Interactive visualization module that converts sensor data into comprehensive 2D environment maps. Provides live feedback of bot exploration and creates persistent topology representations.

## 🔧 Key Responsibilities

- **Coordinate Conversion**: Polar to Cartesian transformation
- **Real-time Display**: Live updating matplotlib visualization
- **Environment Outlining**: Connects scan points to form shapes
- **Auto-scaling**: Dynamic plot adjustment for optimal viewing

## 🚀 Core Features

### Coordinate Transformation
```python
def polar_to_cartesian(self, angle_degrees, distance_cm):
    # Converts sensor data to plottable coordinates
    angle_radians = math.radians(90 - angle_degrees)  # 0° = North
    x = distance_cm * math.cos(angle_radians)
    y = distance_cm * math.sin(angle_radians)
```

### Data Management
- **Point Storage**: Deque-based with configurable limits
- **Scan Sequences**: Preserves complete scan contexts
- **Memory Efficient**: Automatic pruning of old data

### Visualization Elements
```python
# Multiple visualization layers
self.scatter_plot    # Individual obstacle points (blue)
self.line_plots      # Environment outlines (green) 
self.bot_position    # Current bot location (red)
```

## 📊 Display Features

### Auto-scaling System
- **Dynamic Boundaries**: Automatically adjusts to data range
- **Smart Margins**: 10% padding with minimum ranges
- **Aspect Ratio**: Equal scaling for accurate representation

### Environment Outlining
- **Point Connection**: Sorts by angle for proper shape formation
- **Polygon Closure**: Connects first/last points for complete shapes
- **Multiple Scans**: Maintains separate sequences for comparison

## 🎨 Visualization Output

**Color Scheme:**
- 🔵 Blue: Individual sensor measurements
- 🟢 Green: Connected environment outlines  
- 🔴 Red: Bot current position

**Plot Features:**
- Grid background for scale reference
- Center axes for orientation
- Legend for element identification
- High-resolution export capability

## ⚡ Quick Start

```python
# Initialize visualizer
visualizer = TopologyVisualizer(max_points=500)

# Add sensor data
visualizer.add_scan_data(scan_data)

# Update display
visualizer.update_display()

# Save to file
visualizer.save_topology_image("map.png")
```

## 📈 Data Statistics

```python
stats = visualizer.get_statistics()
# Returns: total_points, total_scans, coverage_area, coordinate_ranges
```

## 💾 Export Capabilities

- **High-Resolution PNG**: 300 DPI with tight bounding
- **Timestamped Files**: Automatic naming for session management
- **White Background**: Clean, printable outputs
- **Lossless Quality**: Perfect for documentation and analysis

## 🔄 Integration Points

- **SLAM System**: Visualizes occupancy grid results
- **Bot Tracking**: Shows real-time position updates
- **Path Planning**: Displays planned routes and exploration goals

The eyes of the system that transforms raw sensor readings into intuitive, actionable environment maps for both real-time monitoring and post-analysis.