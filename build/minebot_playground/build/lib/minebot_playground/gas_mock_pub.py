import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class GasMock(Node):
    def __init__(self):
        super().__init__('gas_mock')
        self.pub_co2 = self.create_publisher(Float32, 'gas/co2_ppm', 10) #publica os dados de dióxido de carbono
        self.pub_ch4 = self.create_publisher(Float32, 'gas/ch4_ppm', 10) #publica os dados de metano
        self.declare_parameter('rate', 3.0)  # publica de 3 em 3 segundos (frequencia em hz)
        period = 1.0 / max(0.1, float(self.get_parameter('rate').value))
        self.timer = self.create_timer(period, self.tick) #seta o tick para ser chamado, de acordo com o que foi declarado no parâmetro

    def tick(self): #método chamado de 3 em 3 segundos, de acordo com o que foi criado pelo timer
        self.pub_co2.publish(Float32(data=float(int(random.uniform(400, 1200))))) #mockando, gerando dados aleatórios (deve ser lido pelo sensor)
        self.pub_ch4.publish(Float32(data=round(random.uniform(0, 200), 1))) #mockando, gerando dados aleatórios (deve ser lido pelo sensor)

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
