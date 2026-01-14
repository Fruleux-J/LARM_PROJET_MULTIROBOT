import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class Nav2Client(Node):
    def __init__(self):
        super().__init__('nav2_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, yaw=0.0):
        """Envoie un goal à Nav2 et attend la fin"""
        # Créer la pose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Orientation en quaternion (yaw)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 0.0

        # Attendre que l'action soit prête
        self._action_client.wait_for_server()
        self.get_logger().info(f"Sending goal: x={x}, y={y}, yaw={yaw}")
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return
        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        self.get_logger().info(f"Result: {result.status}")

def main(args=None):
    rclpy.init(args=args)
    node = Nav2Client()
    # Exemple : aller en (1.0, 0.0) avec orientation 0 rad
                #response.x = -0.5
            #response.y = -2.15
            #response.z = 0.0
    node.send_goal(-5.3, 0.84, 0.0)
    #node.send_goal(-0.5, -2.15, 0.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
