import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose

rosNode = None

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


def main(args=None):
    rclpy.init(args=args)

    # Création du node publisher
    rosNodePublisher = Node('Bbot3Publisher')
    publisher = rosNodePublisher.create_publisher(NavigateToPose, 'navigate_to_pose', 10)

    # Création du client pour le service
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request()
    minimal_client.get_logger().info(f"Answer from service: {response}")

    # Conversion de la réponse en PoseStamped
    pointTab = response.message.split(" ")
    point = NavigateToPose.Goal()
    point.header.frame_id = "map"
    point.pose.position.x = float(pointTab[0])
    point.pose.position.y = float(pointTab[1])
    point.pose.position.z = float(pointTab[2])
    point.pose.orientation.x = 0.0
    point.pose.orientation.y = 0.0
    point.pose.orientation.z = 0.0
    point.pose.orientation.w = 1.0
    point.header.stamp = rosNodePublisher.get_clock().now().to_msg()

    # ⚡ Attendre qu'au moins un subscriber soit connecté
    while publisher.get_subscription_count() == 0:
        rosNodePublisher.get_logger().info('Waiting for subscribers...')
        rclpy.spin_once(rosNodePublisher, timeout_sec=0.1)

    # Publier le message plusieurs fois pour fiabilité
    for _ in range(5):
        publisher.publish(point)
        rosNodePublisher.get_logger().info(f"Published goal: ({point.pose.position.x}, {point.pose.position.y})")
        rclpy.spin_once(rosNodePublisher, timeout_sec=0.1)
        time.sleep(0.1)

    # Laisser un peu de temps pour que le message soit traité par les subscribers
    time.sleep(0.5)

    minimal_client.destroy_node()
    rosNodePublisher.destroy_node()
    rclpy.shutdown()
    print("Fin du programme")


if __name__ == '__main__':
    main()
