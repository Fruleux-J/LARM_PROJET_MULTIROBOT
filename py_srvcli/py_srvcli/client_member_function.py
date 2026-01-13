import sys

from example_interfaces.srv import AddTwoInts
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time

rosNode= None

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        #self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        self.cli = self.create_client(Trigger, 'Give_package')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Trigger.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    rosNode= Node('Bbot3')
    publish = rosNode.create_publisher(
            PoseStamped, '/goal_pose', 10
        )
    minimal_client = MinimalClientAsync()
    #response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    response = minimal_client.send_request()
    minimal_client.get_logger().info(f"Answer : {response}")
    pointTab = response.message.split(" ")
    point = PoseStamped()

    point.header.stamp = rosNode.get_clock().now().to_msg()
    point.header.frame_id = "map" 


    point.pose.position.x = float(pointTab[0])
    point.pose.position.y = float(pointTab[1])
    point.pose.position.z = float(pointTab[2])

    point.pose.orientation.x = 0.0
    point.pose.orientation.y = 0.0
    point.pose.orientation.z = 0.0
    point.pose.orientation.w = 1.0
    publish.publish(point)
    time.sleep(10)
    rclpy.spin_once(rosNode, timeout_sec=0.5)
    #print(response.message.split(" "))
    #print(type(response.message))
    minimal_client.destroy_node()
    rosNode.destroy_node()
    rclpy.shutdown()

'''
(-4.62, -2.40) to (-2.95, 1.51) chemin A
(1.55, 0.0) to (-0.5 -2.15) chemin B


'''

if __name__ == '__main__':
    main()