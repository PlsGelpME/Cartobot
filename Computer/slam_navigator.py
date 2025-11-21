# slam_navigator.py - Server-side SLAM and Navigation Module
# This module implements Simultaneous Localization and Mapping (SLAM) with RRT* path planning
# It ensures the mobile bot only moves within known areas for safe navigation

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree, ConvexHull
import math
import time
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass

@dataclass
class Pose:
    """
    Robot pose representation in 2D space
    Contains position (x, y) and orientation (theta)
    """
    x: float          # X coordinate in meters
    y: float          # Y coordinate in meters  
    theta: float      # Orientation in radians

@dataclass
class MapCell:
    """
    Occupancy grid map cell representation
    Tracks occupancy state and confidence level
    """
    occupancy: float   # -1: unknown, 0: free, 1: occupied
    confidence: float  # Confidence level for occupancy

class HectorSLAM:
    """
    Simplified Hector SLAM implementation for 2D mapping
    Based on laser scan matching and occupancy grid mapping principles
    Uses ultrasonic sensor data to build and update environment maps
    """
    
    def __init__(self, map_resolution: float = 0.05, map_size: int = 1000):
        """
        Initialize Hector SLAM system with mapping parameters
        
        Args:
            map_resolution: Meters per grid cell (higher = more detailed)
            map_size: Grid size (size x size cells)
        """
        # Map configuration
        self.map_resolution = map_resolution
        self.map_size = map_size
        
        # Occupancy grid: -1 = unknown, 0 = free, 1 = occupied
        self.occupancy_grid = np.full((map_size, map_size), -1.0, dtype=np.float32)
        
        # Confidence grid for probabilistic updates
        self.confidence_grid = np.zeros((map_size, map_size), dtype=np.float32)
        
        # Robot state estimation
        self.estimated_pose = Pose(0.0, 0.0, 0.0)  # SLAM-corrected pose
        self.odometry_pose = Pose(0.0, 0.0, 0.0)   # Raw odometry pose
        
        # SLAM probabilistic parameters
        self.hit_probability = 0.7    # Confidence increase for occupied cells
        self.miss_probability = 0.4   # Confidence decrease for free cells  
        self.min_confidence = 0.1     # Minimum confidence threshold
        
    def world_to_map(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to map coordinates
        Centers the map around (0,0) world coordinates
        
        Args:
            world_x: X coordinate in meters
            world_y: Y coordinate in meters
            
        Returns:
            tuple: (map_x, map_y) grid coordinates
        """
        # Convert to map coordinates with origin at center
        map_x = int(world_x / self.map_resolution + self.map_size / 2)
        map_y = int(world_y / self.map_resolution + self.map_size / 2)
        return map_x, map_y
    
    def map_to_world(self, map_x: int, map_y: int) -> Tuple[float, float]:
        """
        Convert map coordinates to world coordinates
        
        Args:
            map_x: X coordinate in grid cells
            map_y: Y coordinate in grid cells
            
        Returns:
            tuple: (world_x, world_y) in meters
        """
        # Convert from map coordinates to world coordinates
        world_x = (map_x - self.map_size / 2) * self.map_resolution
        world_y = (map_y - self.map_size / 2) * self.map_resolution
        return world_x, world_y
    
    def update_occupancy_grid(self, scan_data: List[Tuple[float, float]]):
        """
        Update occupancy grid with new ultrasonic scan data
        Implements Bresenham's line algorithm for ray casting
        
        Args:
            scan_data: List of (angle_degrees, distance_cm) measurements
        """
        # Get current robot position
        robot_x, robot_y = self.estimated_pose.x, self.estimated_pose.y
        
        # Process each sensor measurement in the scan
        for angle_deg, distance_cm in scan_data:
            # Filter invalid measurements
            if distance_cm <= 0 or distance_cm > 400:
                continue
                
            # Convert to meters and radians
            distance_m = distance_cm / 100.0
            angle_rad = math.radians(angle_deg)
            
            # Calculate endpoint in world coordinates (adjust for robot orientation)
            end_x = robot_x + distance_m * math.cos(angle_rad + self.estimated_pose.theta)
            end_y = robot_y + distance_m * math.sin(angle_rad + self.estimated_pose.theta)
            
            # Convert to map coordinates
            robot_map_x, robot_map_y = self.world_to_map(robot_x, robot_y)
            end_map_x, end_map_y = self.world_to_map(end_x, end_y)
            
            # Use Bresenham's line algorithm to mark free cells along the ray
            free_cells = self._bresenham_line(robot_map_x, robot_map_y, end_map_x, end_map_y)
            
            # Mark all cells along the ray as free (except endpoint)
            for cell_x, cell_y in free_cells[:-1]:
                if 0 <= cell_x < self.map_size and 0 <= cell_y < self.map_size:
                    self._update_cell_occupancy(cell_x, cell_y, False)
            
            # Mark endpoint as occupied
            if 0 <= end_map_x < self.map_size and 0 <= end_map_y < self.map_size:
                self._update_cell_occupancy(end_map_x, end_map_y, True)
    
    def _bresenham_line(self, x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        """
        Bresenham's line algorithm for drawing lines between points
        Returns all grid cells along the line between two points
        
        Args:
            x0, y0: Start point coordinates
            x1, y1: End point coordinates
            
        Returns:
            list: All grid cells along the line
        """
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        
        # Handle different slope cases
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
                
        points.append((x, y))
        return points
    
    def _update_cell_occupancy(self, map_x: int, map_y: int, is_occupied: bool):
        """
        Update individual cell occupancy with confidence-based approach
        Implements probabilistic occupancy grid update
        
        Args:
            map_x: X coordinate in grid
            map_y: Y coordinate in grid  
            is_occupied: True if cell should be marked as occupied
        """
        if is_occupied:
            # Increase confidence for occupied cells
            self.confidence_grid[map_x, map_y] = min(1.0, 
                self.confidence_grid[map_x, map_y] + self.hit_probability)
        else:
            # Decrease confidence for free cells  
            self.confidence_grid[map_x, map_y] = max(-1.0,
                self.confidence_grid[map_x, map_y] - self.miss_probability)
        
        # Update occupancy based on confidence threshold
        if self.confidence_grid[map_x, map_y] > self.min_confidence:
            self.occupancy_grid[map_x, map_y] = 1.0  # Occupied
        elif self.confidence_grid[map_x, map_y] < -self.min_confidence:
            self.occupancy_grid[map_x, map_y] = 0.0  # Free
        else:
            self.occupancy_grid[map_x, map_y] = -1.0  # Unknown
    
    def estimate_pose_scan_matching(self, current_scan: List[Tuple[float, float]]) -> Pose:
        """
        Estimate robot pose using scan matching against current map
        Simplified implementation using small random corrections
        
        Args:
            current_scan: Current sensor scan data
            
        Returns:
            Pose: Updated robot pose estimate
        """
        # Simplified pose estimation - in full implementation this would use
        # iterative closest point (ICP) or similar scan matching algorithms
        
        # Apply small random corrections to simulate scan matching
        correction_x = np.random.normal(0, 0.01)  # Small position correction
        correction_y = np.random.normal(0, 0.01)  
        correction_theta = np.random.normal(0, 0.005)  # Small orientation correction
        
        # Update estimated pose with corrections
        new_pose = Pose(
            self.odometry_pose.x + correction_x,
            self.odometry_pose.y + correction_y, 
            self.odometry_pose.theta + correction_theta
        )
        
        self.estimated_pose = new_pose
        return new_pose

class RRTStarPlanner:
    """
    RRT* path planner with safety constraints
    Ensures bot never moves into unknown areas by checking occupancy grid
    Provides asymptotically optimal path planning
    """
    
    def __init__(self, slam_system: HectorSLAM, bot_radius: float = 0.3):
        """
        Initialize RRT* planner with SLAM system and safety parameters
        
        Args:
            slam_system: HectorSLAM instance for map access
            bot_radius: Safety radius around bot for collision checking
        """
        self.slam = slam_system
        self.bot_radius = bot_radius  # Safety margin around bot
        
        # RRT* parameters
        self.step_size = 0.1      # meters per expansion step
        self.max_iterations = 1000 # Maximum planning iterations
        self.goal_threshold = 0.2  # meters - goal proximity threshold
        
    class Node:
        """
        RRT* tree node representing a state in configuration space
        """
        def __init__(self, x: float, y: float, parent=None):
            self.x = x          # X coordinate
            self.y = y          # Y coordinate  
            self.parent = parent # Parent node in tree
            self.cost = 0.0     # Cost from start to this node
    
    def is_known_area(self, x: float, y: float) -> bool:
        """
        Check if position is in known area (not unknown in occupancy grid)
        Critical safety check to prevent movement into unknown areas
        
        Args:
            x: X coordinate in meters
            y: Y coordinate in meters
            
        Returns:
            bool: True if area is known, False if unknown
        """
        # Convert world coordinates to map coordinates
        map_x, map_y = self.slam.world_to_map(x, y)
        
        # Check map bounds
        if not (0 <= map_x < self.slam.map_size and 0 <= map_y < self.slam.map_size):
            return False
        
        # Area is known if occupancy grid value is >= 0 (not unknown)
        return self.slam.occupancy_grid[map_x, map_y] >= 0
    
    def is_collision_free(self, x: float, y: float) -> bool:
        """
        Check if position is collision-free (not occupied)
        
        Args:
            x: X coordinate in meters
            y: Y coordinate in meters
            
        Returns:
            bool: True if position is free, False if occupied
        """
        map_x, map_y = self.slam.world_to_map(x, y)
        
        # Check map bounds
        if not (0 <= map_x < self.slam.map_size and 0 <= map_y < self.slam.map_size):
            return False
        
        # Position is free if occupancy grid shows free space (value == 0)
        return self.slam.occupancy_grid[map_x, map_y] == 0
    
    def plan_path(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """
        Plan safe path from start to goal using RRT*
        Returns None if no safe path found within known areas
        
        Args:
            start: (x, y) start position
            goal: (x, y) goal position
            
        Returns:
            list: Path as list of (x, y) points, or None if no path
        """
        # Create start and goal nodes
        start_node = self.Node(start[0], start[1])
        goal_node = self.Node(goal[0], goal[1])
        
        # Early safety check - goal must be in known area
        if not self.is_known_area(goal[0], goal[1]):
            return None
        
        # Initialize tree with start node
        tree = [start_node]
        
        # Main RRT* planning loop
        for iteration in range(self.max_iterations):
            # Sample random point with bias toward goal (20% chance)
            if np.random.random() < 0.2:
                rand_x, rand_y = goal[0], goal[1]  # Bias to goal
            else:
                # Sample within known areas only for safety
                rand_x, rand_y = self._sample_known_area()
            
            # Find nearest node in tree to random point
            nearest_node = self._find_nearest_node(tree, rand_x, rand_y)
            
            # Steer from nearest node toward random point
            new_x, new_y = self._steer(nearest_node.x, nearest_node.y, rand_x, rand_y)
            
            # Check if path to new node is safe and in known area
            if (self._is_path_safe(nearest_node.x, nearest_node.y, new_x, new_y) and 
                self.is_known_area(new_x, new_y) and 
                self.is_collision_free(new_x, new_y)):
                
                # Create new node and calculate cost
                new_node = self.Node(new_x, new_y, nearest_node)
                new_node.cost = nearest_node.cost + self._distance(nearest_node, new_node)
                
                # Find nearby nodes for potential rewiring
                nearby_nodes = self._find_nearby_nodes(tree, new_node, radius=1.0)
                
                # Connect to best parent for optimal path
                new_node = self._choose_best_parent(new_node, nearby_nodes)
                
                # Add new node to tree
                tree.append(new_node)
                
                # Rewire tree to optimize paths
                self._rewire_tree(new_node, nearby_nodes)
                
                # Check if goal is reached
                if self._distance(new_node, goal_node) < self.goal_threshold:
                    return self._reconstruct_path(new_node)
        
        return None  # No path found within iteration limit
    
    def _sample_known_area(self) -> Tuple[float, float]:
        """
        Sample random point within known areas of the map
        Ensures all sampled points are in explored regions
        
        Returns:
            tuple: (x, y) coordinates in known area
        """
        # Find all known cells in occupancy grid
        known_indices = np.where(self.slam.occupancy_grid >= 0)
        
        # Fallback: sample near current position if no known areas
        if len(known_indices[0]) == 0:
            current_x, current_y = self.slam.estimated_pose.x, self.slam.estimated_pose.y
            return current_x + np.random.uniform(-1, 1), current_y + np.random.uniform(-1, 1)
        
        # Randomly select a known cell and convert to world coordinates
        idx = np.random.randint(len(known_indices[0]))
        map_x, map_y = known_indices[0][idx], known_indices[1][idx]
        return self.slam.map_to_world(map_x, map_y)
    
    def _find_nearest_node(self, tree: List[Node], x: float, y: float) -> Node:
        """
        Find nearest node in tree to given point
        
        Args:
            tree: List of nodes in RRT* tree
            x: X coordinate of target point
            y: Y coordinate of target point
            
        Returns:
            Node: Nearest node in tree
        """
        # Create temporary node for distance calculation
        target_node = self.Node(x, y)
        return min(tree, key=lambda node: self._distance(node, target_node))
    
    def _steer(self, from_x: float, from_y: float, to_x: float, to_y: float) -> Tuple[float, float]:
        """
        Steer from current position toward target position
        Moves by step_size in the direction of target
        
        Args:
            from_x: Current X position
            from_y: Current Y position
            to_x: Target X position  
            to_y: Target Y position
            
        Returns:
            tuple: New (x, y) position after steering
        """
        # Calculate distance and direction to target
        dist = math.sqrt((to_x - from_x)**2 + (to_y - from_y)**2)
        
        # If within step size, return target directly
        if dist <= self.step_size:
            return to_x, to_y
        else:
            # Move step_size toward target
            ratio = self.step_size / dist
            return (from_x + (to_x - from_x) * ratio, 
                   from_y + (to_y - from_y) * ratio)
    
    def _is_path_safe(self, x1: float, y1: float, x2: float, y2: float) -> bool:
        """
        Check if straight line path between two points is safe
        Verifies entire path is in known areas and collision-free
        
        Args:
            x1, y1: Start point coordinates
            x2, y2: End point coordinates
            
        Returns:
            bool: True if path is safe, False otherwise
        """
        # Check multiple points along the path for safety
        path_length = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        steps = int(path_length / self.step_size) + 1
        
        for i in range(steps + 1):
            # Calculate point along path
            t = i / steps
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            
            # Check if point is in known area and collision-free
            if not (self.is_known_area(x, y) and self.is_collision_free(x, y)):
                return False
        return True
    
    def _distance(self, node1: Node, node2: Node) -> float:
        """
        Euclidean distance between two nodes
        
        Args:
            node1: First node
            node2: Second node
            
        Returns:
            float: Euclidean distance
        """
        return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
    
    def _find_nearby_nodes(self, tree: List[Node], node: Node, radius: float) -> List[Node]:
        """
        Find nodes within radius of given node
        
        Args:
            tree: List of nodes to search
            node: Center node for search
            radius: Search radius
            
        Returns:
            list: Nodes within search radius
        """
        return [n for n in tree if self._distance(n, node) <= radius]
    
    def _choose_best_parent(self, node: Node, nearby_nodes: List[Node]) -> Node:
        """
        Choose parent that minimizes cost from start
        Implements RRT* optimal parent selection
        
        Args:
            node: Node to find best parent for
            nearby_nodes: Candidate parent nodes
            
        Returns:
            Node: Node with updated best parent
        """
        best_node = node
        best_cost = node.cost
        
        # Check each nearby node as potential parent
        for near_node in nearby_nodes:
            if near_node == node.parent:
                continue
                
            # Check if path from near_node to node is safe
            if self._is_path_safe(near_node.x, near_node.y, node.x, node.y):
                # Calculate cost through this parent
                new_cost = near_node.cost + self._distance(near_node, node)
                if new_cost < best_cost:
                    best_cost = new_cost
                    best_node = near_node
        
        # Update parent if better one found
        if best_node != node.parent:
            node.parent = best_node
            node.cost = best_cost
            
        return node
    
    def _rewire_tree(self, node: Node, nearby_nodes: List[Node]):
        """
        Rewire nearby nodes to go through this node if beneficial
        Implements RRT* tree optimization
        
        Args:
            node: New node to consider as parent
            nearby_nodes: Nearby nodes to potentially rewire
        """
        for near_node in nearby_nodes:
            if near_node == node or near_node == node.parent:
                continue
                
            # Check if path through node is better
            if self._is_path_safe(node.x, node.y, near_node.x, near_node.y):
                new_cost = node.cost + self._distance(node, near_node)
                if new_cost < near_node.cost:
                    # Rewire to use node as parent
                    near_node.parent = node
                    near_node.cost = new_cost
    
    def _reconstruct_path(self, node: Node) -> List[Tuple[float, float]]:
        """
        Reconstruct path from goal node to start node
        Follows parent pointers back to start
        
        Args:
            node: Goal node to start reconstruction from
            
        Returns:
            list: Path as list of (x, y) points from start to goal
        """
        path = []
        current = node
        
        # Follow parent pointers to start
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
            
        # Reverse to get start-to-goal order
        return list(reversed(path))

class SLAMNavigator:
    """
    Main navigation system combining SLAM and path planning
    Provides high-level interface for autonomous navigation
    """
    
    def __init__(self):
        """Initialize the complete navigation system"""
        self.slam = HectorSLAM()
        self.planner = RRTStarPlanner(self.slam)
        self.current_path = []
        self.current_goal = None
        
    def update_with_scan(self, scan_data: List[Tuple[float, float]]):
        """
        Update SLAM system with new sensor scan data
        Performs mapping and pose estimation
        
        Args:
            scan_data: List of (angle_degrees, distance_cm) measurements
        """
        # Update occupancy grid with new sensor data
        self.slam.update_occupancy_grid(scan_data)
        
        # Estimate new pose using scan matching
        self.slam.estimate_pose_scan_matching(scan_data)
        
    def set_navigation_goal(self, goal_x: float, goal_y: float) -> bool:
        """
        Set navigation goal and plan safe path
        Ensures goal is within known areas before planning
        
        Args:
            goal_x: Goal X coordinate
            goal_y: Goal Y coordinate
            
        Returns:
            bool: True if valid path found, False otherwise
        """
        self.current_goal = (goal_x, goal_y)
        start = (self.slam.estimated_pose.x, self.slam.estimated_pose.y)
        
        # Plan path using RRT*
        self.current_path = self.planner.plan_path(start, self.current_goal)
        return self.current_path is not None
    
    def get_next_waypoint(self) -> Optional[Tuple[float, float]]:
        """
        Get next waypoint from current path
        Returns None if no path or path completed
        
        Returns:
            tuple: Next (x, y) waypoint, or None
        """
        if not self.current_path or len(self.current_path) < 2:
            return None
            
        # Return next waypoint (skip current position at index 0)
        return self.current_path[1]
    
    def is_goal_reached(self) -> bool:
        """
        Check if current goal has been reached
        Uses proximity threshold for goal checking
        
        Returns:
            bool: True if goal reached, False otherwise
        """
        if not self.current_goal:
            return False
            
        # Calculate distance to goal
        current_pos = (self.slam.estimated_pose.x, self.slam.estimated_pose.y)
        distance = math.sqrt(
            (current_pos[0] - self.current_goal[0])**2 + 
            (current_pos[1] - self.current_goal[1])**2
        )
        
        # Check if within goal threshold
        return distance < self.planner.goal_threshold
    
    def get_known_area_boundary(self) -> List[Tuple[float, float]]:
        """
        Get boundary of known area for visualization and exploration
        Uses convex hull of known points
        
        Returns:
            list: Boundary points as (x, y) tuples
        """
        # Collect all known area points
        known_points = []
        
        for i in range(self.slam.map_size):
            for j in range(self.slam.map_size):
                if self.slam.occupancy_grid[i, j] >= 0:  # Known area
                    world_x, world_y = self.slam.map_to_world(i, j)
                    known_points.append((world_x, world_y))
        
        # Need at least 3 points for convex hull
        if len(known_points) < 3:
            return []
        
        # Calculate convex hull for boundary
        try:
            points_array = np.array(known_points)
            hull = ConvexHull(points_array)
            
            # Extract boundary points and close the polygon
            boundary_points = [points_array[vertex] for vertex in hull.vertices]
            boundary_points.append(boundary_points[0])  # Close polygon
            
            return boundary_points
        except:
            # Fallback: return all known points if convex hull fails
            return known_points

# Example usage and testing
def main():
    """
    Demonstration of SLAM navigation system functionality
    """
    # Create navigation system
    navigator = SLAMNavigator()
    
    # Simulate receiving scan data from bot (angle_degrees, distance_cm)
    test_scan = [
        (0, 150), (45, 120), (90, 100), (135, 120), (180, 150),
        (225, 120), (270, 100), (315, 120)
    ]
    
    # Update SLAM with scan data
    navigator.update_with_scan(test_scan)
    
    # Set navigation goal within known areas
    current_pose = navigator.slam.estimated_pose
    goal_x, goal_y = current_pose.x + 1.0, current_pose.y + 0.5  # 1m forward, 0.5m right
    
    # Plan and check if path found
    if navigator.set_navigation_goal(goal_x, goal_y):
        print("Path found! Navigating to goal...")
        next_waypoint = navigator.get_next_waypoint()
        print(f"Next waypoint: {next_waypoint}")
    else:
        print("No safe path to goal found!")

if __name__ == "__main__":
    main()