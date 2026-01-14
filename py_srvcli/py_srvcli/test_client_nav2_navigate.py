import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger
import time
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import math

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(Trigger, 'Give_package')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = Trigger.Request()

    def send_request(self):
        """Appelle le service et attend la réponse de manière synchrone."""
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

class Nav2Client(Node):
    def __init__(self):
        super().__init__('nav2_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def get_robot_pose(self):
        """Retourne (x, y, yaw) ou None"""
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            q = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )

            return x, y, yaw

        except Exception:
            return None
        
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
        goal_msg.pose.pose.orientation.w = 1.0

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
        #rclpy.spin_until_future_complete(self, result_future)
        start_time = time.time()
        start_time_temp = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            
            if result_future.done():
                result = result_future.result()
                self.get_logger().info(f"Navigation finished with status: {result.status}")
                return

            if time.time() - start_time_temp > 15:
                pose = self.get_robot_pose()
                if pose:
                    rx, ry, ryaw = pose
                    self.get_logger().info(
                        f"Robot pose: x={rx:.2f}, y={ry:.2f}, yaw={ryaw:.2f}"
                    )
                start_time_temp = time.time()
                if math.fabs(rx-x) < 0.1 and math.fabs(ry-y) < 0.1:
                    self.get_logger().warn("Navigation globally reached, canceling goal...")
                    cancel_future = goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(self, cancel_future)
                    self.get_logger().warn("Goal canceled")
                    return
            
            if time.time() - start_time > 45:
                pose = self.get_robot_pose()
                if pose:
                    rx, ry, ryaw = pose
                    self.get_logger().info(
                        f"Robot pose: x={rx:.2f}, y={ry:.2f}, yaw={ryaw:.2f}"
                    )
                self.get_logger().warn("Navigation timeout reached, canceling goal...")
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                self.get_logger().warn("Goal canceled")
                return
        result = result_future.result()
        self.get_logger().info(f"Result: {result.status}")

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request()
    while response != None:
        minimal_client.get_logger().info(f"Answer from service: {response}")
        pointTab = response.message.split(" ")
        node = Nav2Client()
        node.send_goal(float(pointTab[0]), float(pointTab[1]),float(pointTab[2]))
        node.send_goal(-1.97, -0.73, 0.0)
        response = minimal_client.send_request()
        #rajouter une condition pour get la position du robot puis selon la zone, appelle le service
    #node.send_goal return 4 for everything is fine and 6 for an error 
    #add an loop for try again after 6


    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
