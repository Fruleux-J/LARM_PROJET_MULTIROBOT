from example_interfaces.srv import AddTwoInts
#from example_interfaces.msg import package
import rclpy
from random import Random
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger

Tab_Objet = []
print(type(Tab_Objet))
for i in range(10):
    r = Random()
    if r.random() <= 0.5:
        Tab_Objet.append((i, "A"))
    else :
        Tab_Objet.append((i, "B"))
print(Tab_Objet)
print(Tab_Objet[0][0])

class MinimalService(Node):

    def __init__(self):
        global Tab_Objet
        super().__init__('minimal_service')
        #self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.tab_Objet = Tab_Objet
        self.srv = self.create_service(Trigger, 'Give_package', self.give_package_callback) #TODO change the 'any' to give Tab type

    def give_package_callback(self, request, response):
        #response.sum = request.a + request.b
        #self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        if self.tab_Objet[0][1] == "A":
            #TODO change the coord of the area
            #response.x = -5.3
            #response.y = 0.84
            #response.z = 0.0
            response.message = "-5.3 0.84 0.0"
        else:
            #TODO change the coord of the area
            #response.x = -0.5
            #response.y = -2.15
            #response.z = 0.0
            response.message = "-0.5 -2.15 0.0"
        self.tab_Objet.pop(0)
        self.get_logger().info(f"Outgoing request :  f{response}")
        return response

#Coord for arrival
'''
x = 1.55
y = 0.0
z = 0.0
            '''


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()


    '''
    Paquet :
    Id,
    Zone
    '''