'''
ESTE ARQUIVO É COMO SE FOSSE O main.py, MAS SEM ESTAR MODULARIZADO.
A PRINCIPIO, TEM UMA PERFORMANCE UM POUCO MELHOR, DEVIDO A ESTRUTURA DO
PROJETO SER VIA INTERFACE GRÁFICA, PORTANTO, TALVEZ POSSA SER RELEVANTE
USÁ-LO, CASO O PROJETO FIQUE PESADO
'''


import tkinter as tk
from PIL import Image, ImageTk
import roslibpy
import base64
import io

# --- Configurações ---
ROS_MASTER_IP = '172.30.220.39'
ROS_MASTER_PORT = 9090
VIDEO_TOPIC = '/usbcam_node/compressed'
CMD_TOPIC = '/cmd_vel'  # TOPICO PARA CONTROLE DO ROBO
GAS_CO2_TOPIC = '/gas/co2_ppm'
GAS_CH4_TOPIC = '/gas/ch4_ppm'


class ROSVideoViewer:
    def __init__(self, window):
        self.window = window
        self.window.title("Visualizador da Mina - Robô")

        # --- frame para os sensores
        self.sensor_frame = tk.Frame(window)
        self.sensor_frame.pack(anchor="ne", padx=10, pady=10)  # canto superior direito

        self.co2_label = tk.Label(self.sensor_frame, text="CO2: -- ppm", font=("Helvetica", 14), fg="red")
        self.co2_label.pack()

        self.ch4_label = tk.Label(self.sensor_frame, text="CH4: -- ppm", font=("Helvetica", 14), fg="green")
        self.ch4_label.pack()

        # --- video
        self.video_label = tk.Label(window)
        self.video_label.pack(pady=10)

        # --- joystick frame
        self.joystick_frame = tk.Frame(window)
        self.joystick_frame.pack(pady=10)

        self.create_joystick(self.joystick_frame)

        # --- ROS
        self.ros_client = roslibpy.Ros(host=ROS_MASTER_IP, port=ROS_MASTER_PORT)
        self.image_subscriber = None
        self.gas_co2_subscriber = None
        self.gas_ch4_subscriber = None
        self.commands_publisher = roslibpy.Topic(self.ros_client, CMD_TOPIC, 'geometry_msgs/Twist')

        # --- ROS callbacks
        self.ros_client.on_ready(self.on_ros_connect)
        self.ros_client.on('error', self.on_ros_error)
        self.ros_client.on('close', self.on_ros_close)

        print(f"Trying to connect in: {ROS_MASTER_IP}:{ROS_MASTER_PORT}...")
        try:
            self.ros_client.run()
        except Exception as e:
            print(f"Failed to connect: {e}")

    # --- ROS callbacks
    def on_ros_connect(self, *args):
        print("Connected to ROS MASTER")
        self.setup_subscribers()

    def on_ros_close(self):
        print("Conexão com o ROS Bridge fechada.")
        if hasattr(self.video_label, 'image') and self.video_label.image:
            self.video_label.config(image='')
            self.video_label.image = None

    def on_ros_error(self, error_message):
        print(f"Failed to connect: {error_message}")

    def setup_subscribers(self):
        if not self.ros_client.is_connected:
            print("Not possible to subscribe in the topic.")
            return
        self.subscribe_image_topic()
        self.subscribe_co2_topic()
        self.subscribe_ch4_topic()

    #subscribe to recieve image data
    def subscribe_image_topic(self):
        self.image_subscriber = roslibpy.Topic(
            self.ros_client,
            VIDEO_TOPIC,
            'sensor_msgs/CompressedImage'
        )
        self.image_subscriber.subscribe(self.receive_image_callback)

    #subscribe to recieve the co2 data
    def subscribe_co2_topic(self):
        self.gas_co2_subscriber = roslibpy.Topic(
            self.ros_client,
            GAS_CO2_TOPIC,
            'std_msgs/Float32'
        )
        self.gas_co2_subscriber.subscribe(self.recieve_co2_data)

    #subscrive to recieve the ch4 data
    def subscribe_ch4_topic(self):
        self.gas_ch4_subscriber = roslibpy.Topic(
            self.ros_client,
            GAS_CH4_TOPIC,
            'std_msgs/Float32'
        )
        self.gas_ch4_subscriber.subscribe(self.recieve_ch4_data)

    def recieve_co2_data(self, message):
        dangerous_co2_quantity = 1000
        co2 = int(message['data'])
        text = self.get_gas_text("CO2", dangerous_co2_quantity, co2)
        self.co2_label.config(text=text)

    def recieve_ch4_data(self, message):
        dangerous_ch4_quantity = 5000
        ch4 = int(message['data'])
        text = self.get_gas_text("CH4", dangerous_ch4_quantity, ch4)
        self.ch4_label.config(text=text)

    def receive_image_callback(self, message):
        base64_data = message['data']
        image_bytes = base64.b64decode(base64_data)
        try:
            image = Image.open(io.BytesIO(image_bytes))
            photo = ImageTk.PhotoImage(image)
            self.video_label.config(image=photo)
            self.video_label.image = photo
        except Exception as e:
            print(f"Error to process the image: {e}")

    def get_gas_text(self, gas, dangerous_quantity, gas_value):
        return f"🚨Índice de {gas} alto: {gas_value}ppm" if gas_value > dangerous_quantity else f"{gas}: {gas_value} ppm"

    # --- joystick ---
    def create_joystick(self, frame):
        btn_size = 6

        # Up
        self.btn_up = tk.Button(frame, text="↑", width=btn_size, height=2,
                                command=lambda: self.send_cmd(1.0, 0.0))
        self.btn_up.grid(row=0, column=1)

        # Left
        self.btn_left = tk.Button(frame, text="←", width=btn_size, height=2,
                                  command=lambda: self.send_cmd(0.0, 1.0))
        self.btn_left.grid(row=1, column=0)

        # Stop
        self.btn_stop = tk.Button(frame, text="●", width=btn_size, height=2,
                                  command=lambda: self.send_cmd(0.0, 0.0))
        self.btn_stop.grid(row=1, column=1)

        # Right
        self.btn_right = tk.Button(frame, text="→", width=btn_size, height=2,
                                   command=lambda: self.send_cmd(0.0, -1.0))
        self.btn_right.grid(row=1, column=2)

        # Backward
        self.btn_down = tk.Button(frame, text="↓", width=btn_size, height=2,
                                  command=lambda: self.send_cmd(-1.0, 0.0))
        self.btn_down.grid(row=2, column=1)


    def send_cmd(self, linear, angular):
        """Publica comando de movimento no ROS"""
        if self.ros_client.is_connected:
            twist = roslibpy.Message({
                'linear': {'x': linear, 'y': 0.0, 'z': 0.0},
                'angular': {'x': 0.0, 'y': 0.0, 'z': angular}
            })
            self.commands_publisher.publish(twist)
            print(f"Sent cmd -> linear: {linear}, angular: {angular}")
        else:
            print("ROS client not connected")

    # --- closing the app ---
    def on_closing(self):
        print("Closing the application...")
        if self.ros_client.is_connected:
            if self.image_subscriber:
                self.image_subscriber.unsubscribe()
            self.commands_publisher.unadvertise()
            self.ros_client.terminate()
        self.window.destroy()


if __name__ == '__main__':
    root = tk.Tk()
    root.geometry("800x700")
    app = ROSVideoViewer(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
