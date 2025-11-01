import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Robot, Motor

try:
    left_motor = Motor(forward=17, backward=27, enable=12)
    right_motor = Motor(forward=22, backward=23, enable=13)
    robo = Robot(left=left_motor, right=right_motor)
    
    print("Robot inicializado com sucesso.")
except Exception as e:
    print(f"ERRO: Robot não inicializado. {e}")
    exit()

class CmdVelRobot(Node):
    def __init__(self):
        super().__init__('cmd_vel_robot_listener')
        self.__linear_value = 0.0
        self.__angular_value = 0.0
        self.subscriber = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
        print("Node ROS 2 iniciado: cmd_vel_robot_listener")

    def listener_callback(self, msg):
        self.__linear_value = max(min(msg.linear.x, 1.0), 0.0)  # Sempre 0..1
        self.__angular_value = max(min(msg.angular.z, 1.0), -1.0)
        self.get_logger().info(f"Recebido -> Linear: {self.__linear_value:.2f}, Angular: {self.__angular_value:.2f}")
        self.process_robot()

    def process_robot(self):
        L = self.__linear_value
        A = self.__angular_value

        left_speed  = L * (1 - max(A, 0))
        right_speed = L * (1 + min(A, 0))  

        left_speed  = max(0.0, min(left_speed, 1.0))
        right_speed = max(0.0, min(right_speed, 1.0))

        robo.left_motor.forward(left_speed)
        robo.right_motor.forward(right_speed)

        self.get_logger().info(f"Motores -> Esquerdo: {left_speed:.2f}, Direito: {right_speed:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRobot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Encerrando pelo usuário.")
    finally:
        print("Parando o robô...")
        robo.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()