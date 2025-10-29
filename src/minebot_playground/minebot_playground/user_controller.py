import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CommandsListenerNode(Node):
    def __init__(self):
        super().__init__('cmd_vel')
        # --- setup linear and angular value
        self.__linear_value = 0
        self.__angular_value = 0
        # --- setup subcription
        self.subscriber = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)

    #callback function for every time it receives a data
    #possible values to receive: Forward, Backward, Left, Right, Stop
    def listener_callback(self, msg):
        self.set_linear_value(data=msg)
        self.set_angular_value(data=msg)

        self.get_logger().info(f'Data received: L:{self.__linear_value}: A:{self.__angular_value}')
        self.get_logger().info(f"Current values: L:{self.__linear_value} A:{self.__angular_value}")
        #self.process_command

    def set_linear_value(self, data):
        self.__linear_value = data.linear.x

    def set_angular_value(self, data):
        self.__angular_value = data.angular.z
    
    #get the current linear velocity in the attribute
    def get_linear_value(self):
        return self.__linear_value
    
    #get the current angular velocity in the attribute
    def get_angular_value(self):
        return self.__angular_value
 
        

def main(args=None):
    rclpy.init(args=args)

    node = CommandsListenerNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
