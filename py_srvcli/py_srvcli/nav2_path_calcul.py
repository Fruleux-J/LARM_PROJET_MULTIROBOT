import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from tf2_ros import Buffer, TransformListener


class Nav2PathEvaluator(Node):

    def __init__(self):
        super().__init__('nav2_path_evaluator')

        self.path_client = ActionClient(
            self,
            ComputePathToPose,
            'compute_path_to_pose'
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            String,
            '/points',
            self.listener_callback,
            10
        )

        # Publisher de distance
        self.distance_pub = self.create_publisher(
            String,
            '/cost',
            10
        )
        self.id_colis = -1
        self.waiting_list = []
        self.path_list = []
        self.iteration = 0
        self.distance = 0.0
        self.send_goal()


    def send_goal(self):
        self.path_client.wait_for_server()


    def path_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Le planner a refusé la requête")
            return

        self.get_logger().info("Chemin accepté, attente du résultat")
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.path_result_callback)


    def path_result_callback(self, future):
        result = future.result().result
        path = result.path

        self.get_logger().info(f"Nombre de points: {len(path.poses)}")

        distance = 0.0
        prev = None

        for pose in path.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y

            if prev is not None:
                dx = x - prev[0]
                dy = y - prev[1]
                distance += (dx**2 + dy**2) ** 0.5

            prev = (x, y)

        self.get_logger().info(f"Distance totale estimée: {distance:.2f} m")
        self.distance += distance
        self.iteration += 1
        if self.iteration == len(self.waiting_list) + 2:
            msg = String()
            msg.data = str(self.id_colis) + " " + str(44)  + " " +str(self.distance) #TODO get the ROS_DOMAIN_ID dynamicly
            self.id_colis = -1
            print(f"distance total : {self.distance}")
            self.iteration = 0
            self.distance_pub.publish(msg)
            self.distance = 0.0
            #rclpy.shutdown()

    def listener_callback(self, msg: String, nodeROS):
        Tab_String = str(msg.data).split(",")
        self.id_colis = Tab_String[0]
        point1 = Tab_String[1]
        point2 = Tab_String[2]
        
        print(point2[1:-1].split(" "))
        print(point2[1:-1].split(" ")[0])

        pathList=[]
        for i in range(len(self.waiting_list)):
            path=ComputePathToPose.Goal()
            path.goal = PoseStamped()
            path.start = PoseStamped()
            if i == 0:
                path.use_start = False
            else:
                path.start.pose.position.x = float(self.waiting_list[i])
                path.start.pose.position.y = float(self.waiting_list[i])
                path.start.pose.orientation.w = 1.0
                path.start.header.frame_id = 'map'
                path.start.header.stamp = self.get_clock().now().to_msg()
            path.goal.pose.position.x = float(self.waiting_list[i])
            path.goal.pose.position.y = float(self.waiting_list[i])
            path.goal.pose.orientation.w = 1.0
            path.goal.header.frame_id = 'map'
            path.goal.header.stamp = self.get_clock().now().to_msg()
            self.send_goal_future = self.path_client.send_goal_async(point)
            self.send_goal_future.add_done_callback(self.path_response_callback)


        path1=ComputePathToPose.Goal()
        
        
        if len(self.waiting_list) != 0:
            path1.start = PoseStamped()
            path1.use_start = True
            path1.start.pose.position.x = self.waiting_list[-1].pose.position.x #attention si c'est vide
            path1.start.pose.position.y = self.waiting_list[-1].pose.position.y
            path1.start.pose.orientation.w = 1.0
        else:
            path1.use_start = False

        path1.goal = PoseStamped()

        path1.goal.pose.position.x = float(point1[1:-1].split(" ")[0])
        path1.goal.pose.position.y = float(point1[1:-1].split(" ")[1])
        path1.goal.pose.orientation.w = 1.0
        pathList.append(path1)
        path2=ComputePathToPose.Goal()
        path2.use_start = True

        path2.start = PoseStamped()
        path2.goal = PoseStamped()

        path2.start.pose.position.x = float(point1[1:-1].split(" ")[0])
        path2.start.pose.position.y = float(point1[1:-1].split(" ")[1])
        path2.start.pose.orientation.w = 1.0

        path2.goal.pose.position.x = float(point2[1:-1].split(" ")[0])
        path2.goal.pose.position.y = float(point2[1:-1].split(" ")[1])
        path2.goal.pose.orientation.w = 1.0
        pathList.append(path2)

        for point in pathList:
            if point.use_start == True:
                point.start.header.frame_id = 'map'
                point.start.header.stamp = self.get_clock().now().to_msg()
            point.goal.header.frame_id = 'map'
            point.goal.header.stamp = self.get_clock().now().to_msg()
            self.send_goal_future = self.path_client.send_goal_async(point)
            self.send_goal_future.add_done_callback(self.path_response_callback)
        #return response #je pense qu'il faut lancer le calcul ici pour la distance 

def main(args=None):
    rclpy.init(args=args)
    node = Nav2PathEvaluator()
    rclpy.spin(node)
    rclpy.shutdown()
    #node = ROSListener()
    #TODO Tester le listener, prendre en compte si c'est le premier ou non point, faire en sorte que le listener calcul le path
    '''goal = ComputePathToPose.Goal()
    waiting_list = []
    Point=ComputePathToPose.Goal()
    Point.goal = PoseStamped()
    #goal.goal.header.frame_id = 'map'
    #goal.goal.header.stamp = node.get_clock().now().to_msg()

    Point.goal.pose.position.x = 2.0
    Point.goal.pose.position.y = 1.0
    Point.goal.pose.orientation.w = 1.0
    Point.use_start = False  # pose actuelle du robot
    node.waiting_list.append(Point)
    #print(waiting_list)
    Point=ComputePathToPose.Goal()
    Point.use_start = True

    Point.start = PoseStamped()
    Point.goal = PoseStamped()

    Point.start.pose.position.x = 2.0
    Point.start.pose.position.y = 1.0
    Point.start.pose.orientation.w = 1.0

    Point.goal.pose.position.x = 3.0
    Point.goal.pose.position.y = -1.0
    Point.goal.pose.orientation.w = 1.0
    node.waiting_list.append(Point)'''

    #print(waiting_list)

    #Je subscribe, je reçois un point, je calcule mon coût pour ma file d'attente actuelle, 
    #Je rajoute mon point à la distance, je renvois le coût total
    #Si je reçois sur un autre truc  
    rclpy.spin(node)
    node.get_logger().info("Envoi de la requête de calcul de chemin")
    for point in node.waiting_list:
        if point.use_start == True:
            point.start.header.frame_id = 'map'
            point.start.header.stamp = node.get_clock().now().to_msg()
        point.goal.header.frame_id = 'map'
        point.goal.header.stamp = node.get_clock().now().to_msg()
        node.send_goal_future = node.path_client.send_goal_async(point)
        node.send_goal_future.add_done_callback(node.path_response_callback)
    #ajouter un point pour 
    
    #
