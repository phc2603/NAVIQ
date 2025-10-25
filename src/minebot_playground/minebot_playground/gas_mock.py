import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class GasMock(Node):
    def __init__(self):
        super().__init__('gas_mock')
        self.get_logger().info("chegou")

        #--- setup co2 and ch4 attributes
        self.__co2_data = 0
        self.__ch4_data = 0

        #--- setup publishers
        self.pub_co2 = self.create_publisher(Float32, 'gas/co2_ppm', 10) 
        self.pub_ch4 = self.create_publisher(Float32, 'gas/ch4_ppm', 10) 

        #mocking to publish every time, while doest not have a trigger
        self.declare_parameter('rate', 0.01)  # define the frequency of publishing (0.01 = 10sec)
        period = 1.0 / max(0.1, float(self.get_parameter('rate').value))
        self.timer = self.create_timer(period, self.tick) 

    def tick(self):
        #--------------------mocking data (should be deleted) --------------------
        self.__co2_data = float(int(random.uniform(400, 1200)))
        self.__ch4_data = float(int(random.uniform(0, 200)))
        #-----------------------------------------------------
        self.get_logger().info(f"publishing: {self.__co2_data}; {self.__ch4_data}")
        self.pub_co2.publish(Float32(data=self.__co2_data)) #mocking data, should be read by the sensor
        self.pub_ch4.publish(Float32(data=self.__ch4_data)) #mocking data, should be read by the sensor

    #method to set co2 attribue (should be called in the rasberry loop)
    def set_co2_data(self, co2FloatData):
        self.__co2_data = co2FloatData
    
    #method to set ch4 attribue (should be called in the rasberry loop)
    def set_ch4_data(self, ch4FloatData):
        self.__ch4_data = ch4FloatData

def main():
    rclpy.init()
    n = GasMock()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
