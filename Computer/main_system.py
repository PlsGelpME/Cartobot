# main_system.py - Central Computer Main Control System
# This module integrates all server-side components and runs the complete autonomous navigation system
# Coordinates communication, visualization, and path planning for the mobile bot

import time
import threading
import json
import math
import logging

# Import our custom modules for system integration
from server_comms import ServerComms
from topology_visualizer import TopologyVisualizer
from slam_navigator import SLAMNavigator, Pose
host = '0.0.0.0'#ip address
port = 5000 #port number
class AutonomousNavigationSystem:
    """
    Main system class that integrates all server-side modules
    Coordinates communication, visualization, and navigation for autonomous operation
    """
    
    def __init__(self, host, port):
        """
        Initialize the complete autonomous navigation system
        
        Args:
            host: Server host address (0.0.0.0 for all interfaces)
            port: Server port number for bot communications
        """
        # Initialize all system modules
        self.server_comms = ServerComms(host=host, port=port)  # Handles bot communication
        self.visualizer = TopologyVisualizer(max_points=500, update_interval=1000)  # Environment visualization
        self.navigator = SLAMNavigator()  # SLAM and path planning
        
        # System state tracking
        self.connected_bots = {}  # Dictionary of connected bots and their status
        self.running = False  # Main system control flag
        self.active_bot = None  # Currently controlled bot identifier
        self.current_mission = None  # Current navigation mission data
        
        # Navigation parameters for autonomous operation
        self.exploration_goal_distance = 1.0  # meters - default exploration distance
        self.safety_margin = 0.3  # meters - safety buffer around obstacles
        
        # Setup system-wide logging
        self.setup_logging()
        
        # Register callback functions for bot communications
        self.setup_callbacks()
        
    def setup_logging(self):
        """
        Configure comprehensive logging for the entire system
        Logs to both file and console for debugging and monitoring
        """
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler('autonomous_system.log'),  # Persistent log file
                logging.StreamHandler()  # Console output
            ]
        )
        self.logger = logging.getLogger('AutonomousSystem')
        
    def setup_callbacks(self):
        """
        Register callback functions for different types of bot messages
        Enables event-driven processing of bot communications
        """
        self.server_comms.register_callback('system_status', self.handle_system_status)
        self.server_comms.register_callback('sensor_data', self.handle_sensor_data)
        self.server_comms.register_callback('movement_complete', self.handle_movement_complete)
        
    def handle_system_status(self, bot_id, data):
        """
        Handle system status messages from connected bots
        Updates bot tracking and initiates autonomous operation
        
        Args:
            bot_id: Unique identifier of the sending bot
            data: System status data from the bot
        """
        self.logger.info(f"System status from {bot_id}: {data.get('message', 'No message')}")
        
        # Store bot information in connected bots dictionary
        self.connected_bots[bot_id] = {
            'status': data,
            'last_contact': time.time(),
            'ready': True
        }
        
        # If no active bot is set, assign this bot as active
        if not self.active_bot:
            self.active_bot = bot_id
            self.logger.info(f"Set {bot_id} as active bot")
            
        # Send start signal to begin autonomous operation
        self.send_start_signal(bot_id)
        
    def handle_sensor_data(self, bot_id, data):
        """
        Handle sensor data from bots and update navigation system
        Processes scan data and triggers autonomous movement planning
        
        Args:
            bot_id: Unique identifier of the sending bot
            data: Sensor data containing environment scan information
        """
        # Only process data from the active bot
        if bot_id != self.active_bot:
            return
            
        self.logger.info(f"Processing sensor data from {bot_id}")
        
        # Extract scan data from received message
        data_points = data.get('data_points', [])
        if not data_points:
            self.logger.warning("No data points in sensor data")
            return
            
        # Convert sensor data to format expected by SLAM system
        scan_data = [(point['angle'], point['distance_cm']) for point in data_points]
        
        # Update SLAM system with new environment data
        self.navigator.update_with_scan(scan_data)
        
        # Update visualization with new scan data
        self.visualizer.add_scan_data(scan_data)
        self.visualizer.update_display()
        
        # Plan and execute next autonomous movement
        self.plan_and_execute_next_move(bot_id)
        
    def handle_movement_complete(self, bot_id, data):
        """
        Handle movement completion acknowledgments from bots
        Logs completion and updates system state
        
        Args:
            bot_id: Unique identifier of the sending bot
            data: Movement completion data including execution details
        """
        self.logger.info(f"Movement completed by {bot_id}: {data}")
        
        # Additional processing can be added here:
        # - Update bot position in visualization
        # - Log movement statistics
        # - Trigger next planning cycle if needed
        
    def send_start_signal(self, bot_id):
        """
        Send start signal to begin autonomous operation
        Authorizes the bot to begin the scan-move cycle
        
        Args:
            bot_id: Unique identifier of the target bot
        """
        # Create start instruction packet
        start_instruction = {
            'command': 'start',
            'signal': 'green_flag',
            'message': 'BEGIN_AUTONOMOUS_OPERATION',
            'timestamp': time.time()
        }
        
        # Send start signal to the bot
        if self.server_comms.send_instruction(bot_id, start_instruction):
            self.logger.info(f"Start signal sent to {bot_id}")
        else:
            self.logger.error(f"Failed to send start signal to {bot_id}")
            
    def plan_and_execute_next_move(self, bot_id):
        """
        Plan next movement using SLAM and RRT*, then send instruction to bot
        Implements autonomous exploration and navigation
        
        Args:
            bot_id: Unique identifier of the target bot
        """
        try:
            # Get current bot pose from SLAM system
            current_pose = self.navigator.slam.estimated_pose
            
            # Generate exploration goal using frontier-based exploration
            goal_point = self.generate_exploration_goal(current_pose)
            
            if not goal_point:
                self.logger.warning("No valid exploration goal found")
                return
                
            # Plan safe path to goal using RRT* algorithm
            if self.navigator.set_navigation_goal(goal_point[0], goal_point[1]):
                # Get the next waypoint from the planned path
                next_waypoint = self.navigator.get_next_waypoint()
                
                if next_waypoint:
                    # Calculate movement instruction for the bot
                    movement_instruction = self.calculate_movement_instruction(
                        current_pose, next_waypoint
                    )
                    
                    # Send movement instruction to the bot
                    if self.server_comms.send_instruction(bot_id, movement_instruction):
                        self.logger.info(f"Sent movement instruction to {bot_id}: {movement_instruction}")
                    else:
                        self.logger.error(f"Failed to send movement instruction to {bot_id}")
                else:
                    self.logger.info("Goal reached or no waypoint available")
            else:
                self.logger.warning("No safe path to exploration goal found")
                
        except Exception as e:
            self.logger.error(f"Error in planning and execution: {e}")
            
    def generate_exploration_goal(self, current_pose):
        """
        Generate exploration goal using frontier-based exploration
        Finds safe exploration targets within known areas
        
        Args:
            current_pose: Current bot position and orientation
            
        Returns:
            Optional tuple: (x, y) coordinates of exploration goal, or None if no valid goal
        """
        try:
            # Get known area boundary from SLAM system
            boundary = self.navigator.get_known_area_boundary()
            if not boundary:
                # If no boundary yet, explore in front of the bot
                goal_x = current_pose.x + self.exploration_goal_distance * math.cos(current_pose.theta)
                goal_y = current_pose.y + self.exploration_goal_distance * math.sin(current_pose.theta)
                return goal_x, goal_y
                
            # Find the farthest reachable point on the boundary from current position
            best_point = None
            max_distance = 0
            
            for point in boundary:
                if len(point) != 2:
                    continue
                    
                point_x, point_y = point
                distance = math.sqrt(
                    (point_x - current_pose.x)**2 + (point_y - current_pose.y)**2
                )
                
                # Check if point is within reasonable distance and in known area
                if (0.5 <= distance <= self.exploration_goal_distance and 
                    self.navigator.planner.is_known_area(point_x, point_y)):
                    
                    if distance > max_distance:
                        max_distance = distance
                        best_point = (point_x, point_y)
            
            # If no good boundary point, explore in current direction
            if not best_point:
                goal_x = current_pose.x + self.exploration_goal_distance * math.cos(current_pose.theta)
                goal_y = current_pose.y + self.exploration_goal_distance * math.sin(current_pose.theta)
                
                # Ensure goal is in known area
                if self.navigator.planner.is_known_area(goal_x, goal_y):
                    return goal_x, goal_y
                else:
                    return None
                    
            return best_point
            
        except Exception as e:
            self.logger.error(f"Error generating exploration goal: {e}")
            return None
            
    def calculate_movement_instruction(self, current_pose: Pose, waypoint: Tuple[float, float]) -> Dict:
        """
        Calculate movement instruction for the bot based on current pose and target waypoint
        Converts world coordinates to bot-relative movement commands
        
        Args:
            current_pose: Current bot position and orientation
            waypoint: Target waypoint coordinates (x, y)
            
        Returns:
            Dict: Movement instruction with distance and angle
        """
        # Calculate relative position from current pose to waypoint
        dx = waypoint[0] - current_pose.x
        dy = waypoint[1] - current_pose.y
        
        # Calculate distance and target angle
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        # Calculate angle difference relative to current orientation
        angle_diff = target_angle - current_pose.theta
        
        # Normalize angle to [-pi, pi] range for shortest rotation
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
            
        # Convert to degrees for bot instruction and centimeters for distance
        angle_degrees = math.degrees(angle_diff)
        distance_cm = distance * 100  # Convert meters to centimeters
        
        # Create movement instruction with safety limits
        instruction = {
            'distance': max(10, min(distance_cm, 100)),  # Limit distance for safety (10-100cm)
            'angle': angle_degrees,
            'command': 'move_with_angle',
            'timestamp': time.time(),
            'waypoint': {
                'x': waypoint[0],
                'y': waypoint[1]
            }
        }
        
        return instruction
        
    def start_system(self):
        """
        Start the complete autonomous navigation system
        Initializes all components and begins operation
        
        Returns:
            bool: True if system started successfully, False otherwise
        """
        try:
            self.logger.info("Starting Autonomous Navigation System...")
            self.running = True
            
            # Start server communications in separate thread
            if not self.server_comms.start_server():
                self.logger.error("Failed to start server communications")
                return False
                
            self.logger.info("Server communications started successfully")
            
            # Start visualization in separate thread
            visualization_thread = threading.Thread(target=self.run_visualization)
            visualization_thread.daemon = True
            visualization_thread.start()
            
            self.logger.info("Visualization system started")
            
            # Start system monitoring in separate thread
            monitoring_thread = threading.Thread(target=self.system_monitor)
            monitoring_thread.daemon = True
            monitoring_thread.start()
            
            self.logger.info("System monitor started")
            
            # Start command interface (runs in main thread)
            self.command_interface()
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to start system: {e}")
            return False
            
    def run_visualization(self):
        """
        Run the visualization system in a separate thread
        Handles real-time environment display updates
        """
        try:
            self.visualizer.show_plot()
        except Exception as e:
            self.logger.error(f"Visualization error: {e}")
            
    def system_monitor(self):
        """
        Monitor system health and connected bots
        Runs continuously in background thread
        """
        while self.running:
            try:
                # Check bot connection status
                current_time = time.time()
                disconnected_bots = []
                
                for bot_id, bot_info in self.connected_bots.items():
                    # Check if bot hasn't communicated in 30 seconds
                    if current_time - bot_info['last_contact'] > 30:
                        disconnected_bots.append(bot_id)
                        self.logger.warning(f"Bot {bot_id} appears disconnected")
                
                # Remove disconnected bots from tracking
                for bot_id in disconnected_bots:
                    del self.connected_bots[bot_id]
                    if self.active_bot == bot_id:
                        self.active_bot = None  # Clear active bot if disconnected
                
                # Log system status periodically (every 30 seconds)
                if int(time.time()) % 30 == 0:
                    self.log_system_status()
                    
                time.sleep(5)  # Check every 5 seconds
                
            except Exception as e:
                self.logger.error(f"System monitor error: {e}")
                time.sleep(10)  # Longer wait on error
                
    def log_system_status(self):
        """
        Log current system status for monitoring and debugging
        Provides comprehensive system health information
        """
        status = {
            'connected_bots': len(self.connected_bots),
            'active_bot': self.active_bot,
            'total_messages_received': self.server_comms.stats['total_messages_received'],
            'total_messages_sent': self.server_comms.stats['total_messages_sent'],
            'mapped_points': len(self.visualizer.all_points),
            'slam_pose': {
                'x': self.navigator.slam.estimated_pose.x,
                'y': self.navigator.slam.estimated_pose.y,
                'theta': self.navigator.slam.estimated_pose.theta
            }
        }
        
        self.logger.info(f"System Status: {status}")
        
    def command_interface(self):
        """
        Provide interactive command interface for system control
        Allows real-time monitoring and control of the autonomous system
        """
        print("\n" + "="*60)
        print("AUTONOMOUS NAVIGATION SYSTEM - COMMAND INTERFACE")
        print("="*60)
        print("Commands: status, bots, save_map, stop, help")
        print("="*60)
        
        while self.running:
            try:
                command = input("\nANS> ").strip().lower()
                
                if command == 'status':
                    self.show_system_status()
                elif command == 'bots':
                    self.show_connected_bots()
                elif command == 'save_map':
                    self.save_current_map()
                elif command == 'stop':
                    self.stop_system()
                    break
                elif command == 'help':
                    self.show_help()
                elif command == '':
                    continue
                else:
                    print("Unknown command. Type 'help' for available commands.")
                    
            except KeyboardInterrupt:
                print("\nReceived interrupt signal...")
                self.stop_system()
                break
            except Exception as e:
                print(f"Command error: {e}")
                
    def show_system_status(self):
        """
        Display current system status to console
        Provides human-readable system information
        """
        print("\n=== SYSTEM STATUS ===")
        print(f"Running: {self.running}")
        print(f"Connected Bots: {len(self.connected_bots)}")
        print(f"Active Bot: {self.active_bot}")
        
        # Display communication statistics
        comm_stats = self.server_comms.get_communication_stats()
        print(f"Messages Received: {comm_stats['total_messages_received']}")
        print(f"Messages Sent: {comm_stats['total_messages_sent']}")
        
        # Display visualization statistics
        vis_stats = self.visualizer.get_statistics()
        print(f"Mapped Points: {vis_stats['total_points']}")
        print(f"Map Coverage: {vis_stats['coverage_area']:.2f} cm²")
        
        # Display SLAM position information
        slam_pose = self.navigator.slam.estimated_pose
        print(f"SLAM Position: ({slam_pose.x:.2f}, {slam_pose.y:.2f})")
        print(f"SLAM Orientation: {math.degrees(slam_pose.theta):.1f}°")
        
    def show_connected_bots(self):
        """
        Display connected bots information
        Shows detailed status of all connected mobile bots
        """
        print(f"\n=== CONNECTED BOTS ({len(self.connected_bots)}) ===")
        for bot_id, bot_info in self.connected_bots.items():
            status = "ACTIVE" if bot_id == self.active_bot else "STANDBY"
            last_contact = time.time() - bot_info['last_contact']
            print(f"Bot: {bot_id}")
            print(f"  Status: {status}")
            print(f"  Last Contact: {last_contact:.1f} seconds ago")
            print(f"  Ready: {bot_info.get('ready', False)}")
            print("  ---")
            
    def save_current_map(self):
        """
        Save current map visualization to file
        Creates timestamped image files for documentation
        """
        filename = f"navigation_map_{int(time.time())}.png"
        self.visualizer.save_topology_image(filename)
        print(f"Map saved as {filename}")
        
    def show_help(self):
        """
        Display available commands and usage information
        """
        print("\n=== AVAILABLE COMMANDS ===")
        print("status    - Show system status")
        print("bots      - Show connected bots information")
        print("save_map  - Save current map visualization")
        print("stop      - Stop the system gracefully")
        print("help      - Show this help message")
        print("===========================")
        
    def stop_system(self):
        """
        Stop the entire system gracefully
        Performs cleanup and ensures proper shutdown
        """
        self.logger.info("Stopping Autonomous Navigation System...")
        self.running = False
        
        # Stop all components in proper sequence
        self.server_comms.stop_server()
        
        # Save final map state
        self.visualizer.save_topology_image("final_navigation_map.png")
        
        self.logger.info("Autonomous Navigation System stopped successfully")
        print("System stopped successfully.")

def main():
    """
    Main entry point for the autonomous navigation system
    Creates and starts the complete system
    """
    print("Initializing Autonomous Navigation System...")
    
    # Create and start the complete autonomous system
    system = AutonomousNavigationSystem(host, port)
    
    try:
        if system.start_system():
            print("System started successfully. Use the command interface to control the system.")
        else:
            print("Failed to start system. Check logs for details.")
            
    except Exception as e:
        print(f"Fatal error: {e}")
        logging.error(f"Fatal error in main: {e}")

# Application entry point
if __name__ == "__main__":
    main()