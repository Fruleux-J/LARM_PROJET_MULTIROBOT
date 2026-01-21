#!/usr/bin/env python3
"""
RViz Marker Visualizer Node for Multi-Robot Package Delivery System.

Multi-Domain Version: Uses subprocess to subscribe to /robot_pose on multiple
ROS domains and display all robots in a single RViz.

Displays:
- Zone markers (delivery zones A, B, C)
- Home position markers (pickup locations)
- Package status markers (pending packages)
- Robot positions from multiple domains
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose
import json
import threading
import subprocess
import re

# Zone coordinates (delivery destinations) in map frame
ZONES = {
    'A': {'position': (-5.3, 0.84, 0.0), 'color': (1.0, 0.0, 0.0, 0.8)},   # Red
    'B': {'position': (-0.5, -2.15, 0.0), 'color': (0.0, 1.0, 0.0, 0.8)},  # Green
    'C': {'position': (-4.6, -2.15, 0.0), 'color': (0.0, 0.0, 1.0, 0.8)},  # Blue
}

# Home positions (pickup locations)
HOME_POSITIONS = {
    'home_1': {'position': (1.74, -0.46, 0.0), 'color': (1.0, 1.0, 0.0, 0.8)},  # Yellow
    'home_2': {'position': (1.67, 0.61, 0.0), 'color': (1.0, 0.5, 0.0, 0.8)},   # Orange
}

# Robot colors - different color for each robot by domain ID
ROBOT_COLORS = {
    40: (0.8, 0.0, 0.8, 1.0),   # Purple - Robot 40
    44: (0.0, 0.8, 0.8, 1.0),   # Cyan - Robot 44
    28: (1.0, 0.5, 0.0, 1.0),   # Orange - Robot 28
}
DEFAULT_ROBOT_COLOR = (0.5, 0.5, 0.5, 1.0)  # Gray for unknown robots

# Domains to subscribe to
ROBOT_DOMAINS = [40, 44]

# Task states
TASK_STATE_GOING_TO_PICKUP = 'pickup'
TASK_STATE_GOING_TO_DELIVERY = 'delivery'


class DomainBridge:
    """Bridge to receive robot poses from a specific domain via subprocess."""

    def __init__(self, domain_id, pose_callback, logger):
        self.domain_id = domain_id
        self.pose_callback = pose_callback
        self.logger = logger
        self.process = None
        self.running = False
        self.thread = None

    def start(self):
        """Start the subprocess bridge."""
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        """Run subprocess to echo topic and parse output."""
        try:
            # Command to source ROS and echo topic with specific domain
            cmd = f'''bash -c "source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID={self.domain_id} && ros2 topic echo /robot_pose --no-arr"'''

            self.process = subprocess.Popen(
                cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )

            self.logger.info(f'Domain {self.domain_id} bridge started (subprocess)')

            # Parse output
            buffer = []
            for line in self.process.stdout:
                if not self.running:
                    break

                line = line.rstrip()
                if line == '---':
                    # End of message, parse it
                    if buffer:
                        self._parse_message(buffer)
                        buffer = []
                else:
                    buffer.append(line)

        except Exception as e:
            self.logger.error(f'Domain {self.domain_id} bridge error: {e}')
        finally:
            self._cleanup()

    def _parse_message(self, lines):
        """Parse YAML-like output from ros2 topic echo."""
        try:
            data = {}
            current_section = None
            indent_stack = [data]

            for line in lines:
                if not line.strip():
                    continue

                # Count leading spaces
                stripped = line.lstrip()
                indent = len(line) - len(stripped)

                # Parse key: value
                if ':' in stripped:
                    key, _, value = stripped.partition(':')
                    key = key.strip()
                    value = value.strip()

                    if value:
                        # Simple key: value
                        data[key] = self._convert_value(value)
                    else:
                        # Nested section
                        data[key] = {}

            # Extract values
            ros_domain_id = int(data.get('ros_domain_id', self.domain_id))

            # Create PoseStamped
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'

            # Parse position
            for line in lines:
                if 'x:' in line and 'position' not in line and 'orientation' not in line:
                    # Find x, y, z values in order
                    pass

            # Simple regex extraction
            text = '\n'.join(lines)

            # Extract position
            pos_match = re.search(r'position:\s*\n\s*x:\s*([-\d.]+)\s*\n\s*y:\s*([-\d.]+)\s*\n\s*z:\s*([-\d.]+)', text)
            if pos_match:
                pose_stamped.pose.position.x = float(pos_match.group(1))
                pose_stamped.pose.position.y = float(pos_match.group(2))
                pose_stamped.pose.position.z = float(pos_match.group(3))

            # Extract orientation
            ori_match = re.search(r'orientation:\s*\n\s*x:\s*([-\d.]+)\s*\n\s*y:\s*([-\d.]+)\s*\n\s*z:\s*([-\d.]+)\s*\n\s*w:\s*([-\d.]+)', text)
            if ori_match:
                pose_stamped.pose.orientation.x = float(ori_match.group(1))
                pose_stamped.pose.orientation.y = float(ori_match.group(2))
                pose_stamped.pose.orientation.z = float(ori_match.group(3))
                pose_stamped.pose.orientation.w = float(ori_match.group(4))

            # Extract domain ID
            domain_match = re.search(r'ros_domain_id:\s*(\d+)', text)
            if domain_match:
                ros_domain_id = int(domain_match.group(1))

            # Call callback with parsed data
            self.pose_callback(ros_domain_id, pose_stamped)

        except Exception as e:
            self.logger.debug(f'Parse error: {e}')

    def _convert_value(self, value):
        """Convert string value to appropriate type."""
        try:
            if '.' in value:
                return float(value)
            return int(value)
        except ValueError:
            return value

    def _cleanup(self):
        """Clean up subprocess."""
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()

    def stop(self):
        """Stop the bridge."""
        self.running = False
        self._cleanup()
        if self.thread:
            self.thread.join(timeout=2.0)


class WinBidBridge:
    """Bridge to receive win_bid messages from a specific domain via subprocess."""

    def __init__(self, domain_id, win_bid_callback, logger):
        self.domain_id = domain_id
        self.win_bid_callback = win_bid_callback
        self.logger = logger
        self.process = None
        self.running = False
        self.thread = None

    def start(self):
        """Start the subprocess bridge."""
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        """Run subprocess to echo win_bid topic and parse output."""
        try:
            cmd = f'''bash -c "source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID={self.domain_id} && ros2 topic echo /win_bid --no-arr"'''

            self.process = subprocess.Popen(
                cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )

            self.logger.info(f'Domain {self.domain_id} win_bid bridge started')

            buffer = []
            for line in self.process.stdout:
                if not self.running:
                    break

                line = line.rstrip()
                if line == '---':
                    if buffer:
                        self._parse_message(buffer)
                        buffer = []
                else:
                    buffer.append(line)

        except Exception as e:
            self.logger.error(f'Domain {self.domain_id} win_bid bridge error: {e}')
        finally:
            self._cleanup()

    def _parse_message(self, lines):
        """Parse win_bid message (Package type: id, departure, arrival)."""
        try:
            text = '\n'.join(lines)

            # Extract package id
            id_match = re.search(r'^id:\s*(\d+)', text, re.MULTILINE)
            package_id = int(id_match.group(1)) if id_match else -1

            # Extract departure position
            departure_match = re.search(
                r'departure:.*?position:\s*\n\s*x:\s*([-\d.]+)\s*\n\s*y:\s*([-\d.]+)\s*\n\s*z:\s*([-\d.]+)',
                text, re.DOTALL
            )
            departure = None
            if departure_match:
                departure = (
                    float(departure_match.group(1)),
                    float(departure_match.group(2)),
                    float(departure_match.group(3))
                )

            # Extract arrival position
            arrival_match = re.search(
                r'arrival:.*?position:\s*\n\s*x:\s*([-\d.]+)\s*\n\s*y:\s*([-\d.]+)\s*\n\s*z:\s*([-\d.]+)',
                text, re.DOTALL
            )
            arrival = None
            if arrival_match:
                arrival = (
                    float(arrival_match.group(1)),
                    float(arrival_match.group(2)),
                    float(arrival_match.group(3))
                )

            if departure and arrival:
                self.win_bid_callback(self.domain_id, package_id, departure, arrival)

        except Exception as e:
            self.logger.debug(f'Win bid parse error: {e}')

    def _cleanup(self):
        """Clean up subprocess."""
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()

    def stop(self):
        """Stop the bridge."""
        self.running = False
        self._cleanup()
        if self.thread:
            self.thread.join(timeout=2.0)


class MarkerVisualizer(Node):
    def __init__(self):
        super().__init__('marker_visualizer')

        # Publisher for markers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/visualization_markers',
            10
        )

        # Subscriber for package status (on main domain)
        self.status_sub = self.create_subscription(
            String,
            '/package_status',
            self.status_callback,
            10
        )

        # Store current states
        self.package_status = None
        self.robot_poses = {}  # Dict: {robot_id: PoseStamped}
        self.poses_lock = threading.Lock()

        # Robot tasks: {robot_id: {'state': 'pickup'/'delivery', 'package_id': int,
        #                          'pickup': (x,y,z), 'delivery': (x,y,z), 'tasks': [(pickup, delivery), ...]}}
        self.robot_tasks = {}
        self.tasks_lock = threading.Lock()

        # Domain bridges
        self.domain_bridges = []
        self.win_bid_bridges = []

        # Timer for regular marker updates (2 Hz)
        self.create_timer(0.5, self.update_and_publish)

        self.get_logger().info('Multi-Domain Marker Visualizer started')
        self.get_logger().info(f'Subscribing to domains: {ROBOT_DOMAINS}')
        self.get_logger().info('Publishing to: /visualization_markers')

        # Start domain bridges
        self._start_domain_bridges()
        self._start_win_bid_bridges()

    def _start_domain_bridges(self):
        """Start bridges for each robot domain."""
        for domain_id in ROBOT_DOMAINS:
            bridge = DomainBridge(
                domain_id,
                self.robot_pose_callback,
                self.get_logger()
            )
            bridge.start()
            self.domain_bridges.append(bridge)

    def _start_win_bid_bridges(self):
        """Start win_bid bridges for each robot domain."""
        for domain_id in ROBOT_DOMAINS:
            bridge = WinBidBridge(
                domain_id,
                self.win_bid_callback,
                self.get_logger()
            )
            bridge.start()
            self.win_bid_bridges.append(bridge)

    def win_bid_callback(self, robot_id, package_id, departure, arrival):
        """Handle incoming win_bid message - robot assigned a package.

        In the Package message from usine:
        - departure = pickup point (Home 1/2)
        - arrival = delivery zone (Zone A/B/C)
        """
        pickup = departure
        delivery = arrival

        with self.tasks_lock:
            if robot_id not in self.robot_tasks:
                self.robot_tasks[robot_id] = {
                    'state': TASK_STATE_GOING_TO_PICKUP,
                    'package_id': package_id,
                    'pickup': pickup,
                    'delivery': delivery,
                    'tasks': []
                }
            else:
                # Add to task queue
                self.robot_tasks[robot_id]['tasks'].append((package_id, pickup, delivery))

        self.get_logger().info(
            f'Robot {robot_id} assigned package {package_id}: '
            f'pickup ({pickup[0]:.2f}, {pickup[1]:.2f}) -> '
            f'delivery ({delivery[0]:.2f}, {delivery[1]:.2f})'
        )

    def robot_pose_callback(self, robot_id, pose_stamped):
        """Handle incoming robot pose from any domain."""
        with self.poses_lock:
            self.robot_poses[robot_id] = pose_stamped

        # Check if robot reached pickup or delivery point
        self._check_task_progress(robot_id, pose_stamped)

    def _check_task_progress(self, robot_id, pose_stamped):
        """Check if robot has reached pickup or delivery location."""
        threshold = 0.8  # Distance threshold in meters

        with self.tasks_lock:
            if robot_id not in self.robot_tasks:
                return

            task = self.robot_tasks[robot_id]
            robot_x = pose_stamped.pose.position.x
            robot_y = pose_stamped.pose.position.y

            if task['state'] == TASK_STATE_GOING_TO_PICKUP:
                # Check if reached pickup
                pickup = task['pickup']
                dist = ((robot_x - pickup[0])**2 + (robot_y - pickup[1])**2)**0.5
                if dist < threshold:
                    task['state'] = TASK_STATE_GOING_TO_DELIVERY
                    self.get_logger().info(
                        f'Robot {robot_id} PICKED UP package {task["package_id"]}'
                    )

            elif task['state'] == TASK_STATE_GOING_TO_DELIVERY:
                # Check if reached delivery
                delivery = task['delivery']
                dist = ((robot_x - delivery[0])**2 + (robot_y - delivery[1])**2)**0.5
                # Debug: log distance periodically
                self.get_logger().debug(
                    f'Robot {robot_id} dist to delivery ({delivery[0]:.2f}, {delivery[1]:.2f}): {dist:.2f}m'
                )
                if dist < threshold:
                    self.get_logger().info(
                        f'Robot {robot_id} DELIVERED package {task["package_id"]}'
                    )
                    # Check for next task in queue
                    if task['tasks']:
                        next_task = task['tasks'].pop(0)
                        task['package_id'] = next_task[0]
                        task['pickup'] = next_task[1]
                        task['delivery'] = next_task[2]
                        task['state'] = TASK_STATE_GOING_TO_PICKUP
                    else:
                        # No more tasks, remove from tracking
                        del self.robot_tasks[robot_id]

    def status_callback(self, msg):
        """Handle incoming package status updates."""
        try:
            self.package_status = json.loads(msg.data)
            self.get_logger().debug(
                f'Received status: {self.package_status["total_remaining"]} packages remaining'
            )
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse status: {e}')

    def update_and_publish(self):
        """Publish all markers."""
        self.publish_markers()

    def create_zone_marker(self, zone_name, zone_info, marker_id):
        """Create a cube marker for a delivery zone."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'zones'
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = zone_info['position'][0]
        marker.pose.position.y = zone_info['position'][1]
        marker.pose.position.z = 0.1  # Slightly above ground
        marker.pose.orientation.w = 1.0

        # Scale (diameter 0.5m, height 0.2m)
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.2

        # Color
        marker.color.r = zone_info['color'][0]
        marker.color.g = zone_info['color'][1]
        marker.color.b = zone_info['color'][2]
        marker.color.a = zone_info['color'][3]

        return marker

    def create_zone_label(self, zone_name, zone_info, marker_id):
        """Create a text label for a zone."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'zone_labels'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Position (above the zone)
        marker.pose.position.x = zone_info['position'][0]
        marker.pose.position.y = zone_info['position'][1]
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0

        # Text
        marker.text = f'Zone {zone_name}'
        marker.scale.z = 0.3  # Text height

        # White text
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        return marker

    def create_home_marker(self, home_name, home_info, marker_id):
        """Create a cylinder marker for home/pickup position."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'home_positions'
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = home_info['position'][0]
        marker.pose.position.y = home_info['position'][1]
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0

        # Scale
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.2

        # Color
        marker.color.r = home_info['color'][0]
        marker.color.g = home_info['color'][1]
        marker.color.b = home_info['color'][2]
        marker.color.a = home_info['color'][3]

        return marker

    def create_home_label(self, home_name, home_info, marker_id):
        """Create a text label for home position."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'home_labels'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Position (above the home)
        marker.pose.position.x = home_info['position'][0]
        marker.pose.position.y = home_info['position'][1]
        marker.pose.position.z = 0.4
        marker.pose.orientation.w = 1.0

        # Text
        label = 'Pickup 1' if home_name == 'home_1' else 'Pickup 2'
        marker.text = label
        marker.scale.z = 0.2

        # White text
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        return marker

    def create_package_count_marker(self, zone_name, count, position, marker_id):
        """Create a text marker showing package count for a zone."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'package_counts'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Position (above home position)
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0.8
        marker.pose.orientation.w = 1.0

        # Count text
        marker.text = f'{zone_name}: {count} pkgs'
        marker.scale.z = 0.25

        # Cyan text
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        return marker

    def create_package_spheres(self, packages, base_position, zone_id, start_marker_id):
        """Create small sphere markers representing pending packages."""
        markers = []

        # Arrange packages in a grid pattern near home position
        packages_per_row = 5
        spacing = 0.15

        for i, pkg in enumerate(packages[:20]):  # Limit display to 20 packages
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = f'packages_zone_{zone_id}'
            marker.id = start_marker_id + i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Grid position offset from base
            row = i // packages_per_row
            col = i % packages_per_row
            marker.pose.position.x = base_position[0] + (col - 2) * spacing
            marker.pose.position.y = base_position[1] + row * spacing + 0.5
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0

            # Small sphere
            marker.scale.x = 0.08
            marker.scale.y = 0.08
            marker.scale.z = 0.08

            # Color based on category
            category = pkg.get('category', 'C') if isinstance(pkg, dict) else pkg[1]
            if category == 'A':
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
            elif category == 'B':
                marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
            else:  # C
                marker.color.r, marker.color.g, marker.color.b = 0.0, 0.0, 1.0
            marker.color.a = 1.0

            markers.append(marker)

        return markers

    def create_robot_marker(self, pose_stamped, robot_id, marker_id):
        """Create a sphere marker for robot position."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f'robot_{robot_id}'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Position and orientation from pose
        marker.pose = pose_stamped.pose

        # Sphere scale (diameter)
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        # Color based on robot ID
        color = ROBOT_COLORS.get(robot_id, DEFAULT_ROBOT_COLOR)
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        return marker

    def create_robot_label(self, pose_stamped, robot_id, marker_id):
        """Create a text label for robot."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f'robot_label_{robot_id}'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Position above robot
        marker.pose.position.x = pose_stamped.pose.position.x
        marker.pose.position.y = pose_stamped.pose.position.y
        marker.pose.position.z = pose_stamped.pose.position.z + 0.5
        marker.pose.orientation.w = 1.0

        marker.text = f'Robot {robot_id}'
        marker.scale.z = 0.2

        # Color based on robot ID
        color = ROBOT_COLORS.get(robot_id, DEFAULT_ROBOT_COLOR)
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0

        return marker

    def create_carried_package_marker(self, robot_pose, robot_id, package_id, marker_id):
        """Create a small cube marker on robot showing it's carrying a package."""
        markers = []

        # Package cube
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f'carried_package_{robot_id}'
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Position on top of robot
        marker.pose.position.x = robot_pose.pose.position.x
        marker.pose.position.y = robot_pose.pose.position.y
        marker.pose.position.z = 0.4
        marker.pose.orientation.w = 1.0

        # Small cube
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15

        # Brown/orange color for package
        marker.color.r = 0.8
        marker.color.g = 0.5
        marker.color.b = 0.2
        marker.color.a = 1.0
        markers.append(marker)

        # Package ID label
        label = Marker()
        label.header.frame_id = 'map'
        label.header.stamp = self.get_clock().now().to_msg()
        label.ns = f'package_label_{robot_id}'
        label.id = marker_id + 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD

        label.pose.position.x = robot_pose.pose.position.x
        label.pose.position.y = robot_pose.pose.position.y
        label.pose.position.z = 0.6
        label.pose.orientation.w = 1.0

        label.text = f'#{package_id}'
        label.scale.z = 0.15

        # White text
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        markers.append(label)

        return markers

    def publish_markers(self):
        """Publish all markers."""
        marker_array = MarkerArray()
        marker_id = 0

        # 1. Zone markers (delivery destinations)
        for zone_name, zone_info in ZONES.items():
            marker_array.markers.append(
                self.create_zone_marker(zone_name, zone_info, marker_id)
            )
            marker_id += 1
            marker_array.markers.append(
                self.create_zone_label(zone_name, zone_info, marker_id)
            )
            marker_id += 1

        # 2. Home position markers (pickup locations)
        for home_name, home_info in HOME_POSITIONS.items():
            marker_array.markers.append(
                self.create_home_marker(home_name, home_info, marker_id)
            )
            marker_id += 1
            marker_array.markers.append(
                self.create_home_label(home_name, home_info, marker_id)
            )
            marker_id += 1

        # 3. Package status markers (if status available)
        if self.package_status:
            # Zone 0 packages (near home_1)
            zone_0_pkgs = self.package_status.get('zone_0', [])
            marker_array.markers.append(
                self.create_package_count_marker(
                    'Zone0',
                    len(zone_0_pkgs),
                    HOME_POSITIONS['home_1']['position'],
                    marker_id
                )
            )
            marker_id += 1

            pkg_markers = self.create_package_spheres(
                zone_0_pkgs,
                HOME_POSITIONS['home_1']['position'],
                0,
                marker_id
            )
            marker_array.markers.extend(pkg_markers)
            marker_id += len(pkg_markers)

            # Zone 1 packages (near home_2)
            zone_1_pkgs = self.package_status.get('zone_1', [])
            marker_array.markers.append(
                self.create_package_count_marker(
                    'Zone1',
                    len(zone_1_pkgs),
                    HOME_POSITIONS['home_2']['position'],
                    marker_id
                )
            )
            marker_id += 1

            pkg_markers = self.create_package_spheres(
                zone_1_pkgs,
                HOME_POSITIONS['home_2']['position'],
                1,
                marker_id
            )
            marker_array.markers.extend(pkg_markers)
            marker_id += len(pkg_markers)

        # 4. Robot markers (for all robots received from multiple domains)
        with self.poses_lock:
            for robot_id, pose in self.robot_poses.items():
                marker_array.markers.append(
                    self.create_robot_marker(pose, robot_id, marker_id)
                )
                marker_id += 1
                marker_array.markers.append(
                    self.create_robot_label(pose, robot_id, marker_id)
                )
                marker_id += 1

        # 5. Carried package markers (only when robot is delivering)
        with self.tasks_lock:
            robot_poses_copy = {}
            with self.poses_lock:
                robot_poses_copy = dict(self.robot_poses)

            for robot_id, task in self.robot_tasks.items():
                if robot_id not in robot_poses_copy:
                    continue

                robot_pose = robot_poses_copy[robot_id]
                is_pickup = (task['state'] == TASK_STATE_GOING_TO_PICKUP)
                package_id = task['package_id']

                # Show package on robot only when delivering (after pickup)
                if not is_pickup:
                    pkg_markers = self.create_carried_package_marker(
                        robot_pose, robot_id, package_id, marker_id
                    )
                    marker_array.markers.extend(pkg_markers)
                    marker_id += len(pkg_markers)

        # Publish
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop all domain bridges
        for bridge in node.domain_bridges:
            bridge.stop()
        for bridge in node.win_bid_bridges:
            bridge.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
