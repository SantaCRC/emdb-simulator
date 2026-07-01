import rclpy
from rclpy.node import Node
from pynput.keyboard import Key, Listener
from emdb_interfaces.srv import SetPosition, SetDeltaAction

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



class KeyboardDeltaClient(Node):
    def __init__(self):
        super().__init__('keyboard_delta_client')
        self.cli = self.create_client(SetDeltaAction, '/set_delta_action')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /set_delta_action...')

        self.pos_step = 0.05
        self.rot_step = 0.1
        self.listener = Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def send_delta(self, dx=0.0, dy=0.0, dz=0.0, droll=0.0, dpitch=0.0, dyaw=0.0, grasp=0, reset=0):
        req = SetDeltaAction.Request()
        req.dx = dx
        req.dy = dy
        req.dz = dz
        req.droll = droll
        req.dpitch = dpitch
        req.dyaw = dyaw
        req.grasp = grasp
        req.reset = reset
        future = self.cli.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            res = future.result()
            self.get_logger().info(res.message)
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def on_press(self, key):
        try:
            if key == Key.up:
                self.send_delta(dx=-self.pos_step)
            elif key == Key.down:
                self.send_delta(dx=self.pos_step)
            elif key == Key.left:
                self.send_delta(dy=-self.pos_step)
            elif key == Key.right:
                self.send_delta(dy=self.pos_step)
            elif key.char == '.':
                self.send_delta(dz=-self.pos_step)
            elif key.char == ';':
                self.send_delta(dz=self.pos_step)
            elif key.char == 'e':
                self.send_delta(dpitch=-self.rot_step)
            elif key.char == 'r':
                self.send_delta(dpitch=self.rot_step)
            elif key.char == 'y':
                self.send_delta(droll=self.rot_step)
            elif key.char == 'h':
                self.send_delta(droll=-self.rot_step)
            elif key.char == 'p':
                self.send_delta(dyaw=self.rot_step)
            elif key.char == 'o':
                self.send_delta(dyaw=-self.rot_step)
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            if key == Key.space:
                self.send_delta(grasp=1)
            elif key.char == 'q':
                self.send_delta(reset=1)
        except AttributeError:
            pass

def main():
    rclpy.init()
    node = KeyboardDeltaClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()