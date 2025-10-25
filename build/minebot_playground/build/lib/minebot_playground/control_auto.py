import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class AutoController(Node):
    def __init__(self):
        super().__init__('auto_controller') #criando o nó auto_controller
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10) #cria um publisher para o tópico cmd_vel,
        self.declare_parameter('v_lin', 0.2)   # m/s
        self.declare_parameter('w_ang', 0.8)   # rad/s
        self.declare_parameter('t_lin', 2.0)   # s andando
        self.declare_parameter('t_turn', 1.6)  # s girando 90 deg
        self.state = 'forward'
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.loop)  # 10 Hz

    def loop(self):
        v = self.get_parameter('v_lin').value
        w = self.get_parameter('w_ang').value
        t_lin = self.get_parameter('t_lin').value
        t_turn = self.get_parameter('t_turn').value

        dt = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        msg = Twist()
        if self.state == 'forward':
            msg.linear.x = float(v)
            if dt >= t_lin:
                self.state = 'turn'
                self.t0 = self.get_clock().now()
        elif self.state == 'turn':
            msg.angular.z = float(w)
            if dt >= t_turn:
                self.state = 'forward'
                self.t0 = self.get_clock().now()

        self.pub.publish(msg)

def main():
    rclpy.init()
    n = AutoController()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
