import rclpy
from rclpy.node import Node
from pynput.keyboard import Key, Listener
from emdb_interfaces.srv import SetDeltaAction


class KeyboardDeltaClient(Node):
    def __init__(self):
        super().__init__('keyboard_delta_client')
        self.cli = self.create_client(SetDeltaAction, '/set_delta_action')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /set_delta_action...')

        self.pos_step = 0.05
        self.rot_step = 0.1
        self.base_step = 0.1
        self.base_rot_step = 0.2

        self.base_mode = False

        self.listener = Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        self.print_controls()

    def print_controls(self):
        self.get_logger().info('Controles:')
        self.get_logger().info('Modo brazo: flechas XY, . / ; en Z, e-r / y-h / o-p rotaciones')
        self.get_logger().info('b: alternar modo brazo/base')
        self.get_logger().info('Modo base: flechas mover base, o/p girar base')
        self.get_logger().info('space: grasp, q: reset')

    def send_delta(
        self,
        dx=0.0, dy=0.0, dz=0.0,
        droll=0.0, dpitch=0.0, dyaw=0.0,
        base_dx=0.0, base_dy=0.0, base_dyaw=0.0,
        grasp=0, reset=0, toggle_base_mode=0
    ):
        req = SetDeltaAction.Request()
        req.dx = dx
        req.dy = dy
        req.dz = dz
        req.droll = droll
        req.dpitch = dpitch
        req.dyaw = dyaw
        req.base_dx = base_dx
        req.base_dy = base_dy
        req.base_dyaw = base_dyaw
        req.grasp = grasp
        req.reset = reset
        req.toggle_base_mode = toggle_base_mode

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
            if not self.base_mode:
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
                    self.send_delta(droll=self.rot_step)
                elif key.char == 'r':
                    self.send_delta(droll=-self.rot_step)
                elif key.char == 'y':
                    self.send_delta(dpitch=self.rot_step)
                elif key.char == 'h':
                    self.send_delta(dpitch=-self.rot_step)
                elif key.char == 'p':
                    self.send_delta(dyaw=self.rot_step)
                elif key.char == 'o':
                    self.send_delta(dyaw=-self.rot_step)
            else:
                if key == Key.up:
                    self.send_delta(base_dx=self.base_step)
                elif key == Key.down:
                    self.send_delta(base_dx=-self.base_step)
                elif key == Key.left:
                    self.send_delta(base_dy=self.base_step)
                elif key == Key.right:
                    self.send_delta(base_dy=-self.base_step)
                elif key.char == 'o':
                    self.send_delta(base_dyaw=self.base_rot_step)
                elif key.char == 'p':
                    self.send_delta(base_dyaw=-self.base_rot_step)

        except AttributeError:
            pass

    def on_release(self, key):
        try:
            if key == Key.space:
                self.send_delta(grasp=1)
            elif key.char == 'q':
                self.send_delta(reset=1)
            elif key.char == 'b':
                self.base_mode = not self.base_mode
                self.send_delta(toggle_base_mode=1)
                mode = 'BASE' if self.base_mode else 'BRAZO'
                self.get_logger().info(f'Modo cambiado a: {mode}')
        except AttributeError:
            pass


def main():
    rclpy.init()
    node = KeyboardDeltaClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()