import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from emdb_interfaces.srv import SetPosition

class PositionServer(Node):
    def __init__(self):
        super().__init__('position_server')
        self.position = Point()
        self.pub = self.create_publisher(Point, '/target_position', 10)
        self.srv = self.create_service(SetPosition, 'set_position', self.set_position_callback)
        self.timer = self.create_timer(0.1, self.publish_position)

    def set_position_callback(self, request, response):
        self.position.x = request.x
        self.position.y = request.y
        self.position.z = request.z
        response.success = True
        response.message = f'Posición actualizada a ({request.x}, {request.y}, {request.z})'
        self.get_logger().info(response.message)
        return response

    def publish_position(self):
        self.pub.publish(self.position)

def main():
    rclpy.init()
    node = PositionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()