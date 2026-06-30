import rclpy
from rclpy.node import Node
from pynput.keyboard import Key, Listener
from emdb_interfaces.srv import SetPosition

class KeyboardPositionClient(Node):
    def __init__(self):
        super().__init__('keyboard_position_client')

        self.pos = [0.0, 0.0, 0.0]
        self.step = 0.05

        self.cli = self.create_client(SetPosition, 'set_position')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio set_position...')

        self.listener = Listener(on_press=self.on_press)
        self.listener.start()

        self.print_controls()

    def print_controls(self):
        self.get_logger().info('Controles:')
        self.get_logger().info('Flechas: mover en X/Y')
        self.get_logger().info('. / ; : mover en Z')

    def send_position(self):
        req = SetPosition.Request()
        req.x = self.pos[0]
        req.y = self.pos[1]
        req.z = self.pos[2]
        future = self.cli.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'{response.message}')
        except Exception as e:
            self.get_logger().error(f'Error en servicio: {e}')

    def on_press(self, key):
        try:
            if key == Key.up:
                self.pos[0] -= self.step
            elif key == Key.down:
                self.pos[0] += self.step
            elif key == Key.left:
                self.pos[1] -= self.step
            elif key == Key.right:
                self.pos[1] += self.step
            elif key.char == ".":
                self.pos[2] -= self.step
            elif key.char == ";":
                self.pos[2] += self.step
            else:
                return

            self.get_logger().info(
                f'Nueva posición: x={self.pos[0]:.2f}, y={self.pos[1]:.2f}, z={self.pos[2]:.2f}'
            )
            self.send_position()

        except AttributeError:
            pass

def main():
    rclpy.init()
    node = KeyboardPositionClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()