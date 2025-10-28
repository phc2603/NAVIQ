import tkinter as tk
from ROS_Bridge.ROS_Bridge import ROS_Bridge
from joystick.Joystick import Joystick


class GUI:
    def __init__(self, window):
        self.window = window
        self.window.title("Visualizador da Mina - Robô")

        # --- frame para os sensores
        self.sensor_frame = tk.Frame(window)
        self.sensor_frame.pack(anchor="ne", padx=10, pady=10)

        self.co2_label = tk.Label(self.sensor_frame, text="CO2: -- ppm", font=("Helvetica", 14), fg="red")
        self.co2_label.pack()

        self.ch4_label = tk.Label(self.sensor_frame, text="CH4: -- ppm", font=("Helvetica", 14), fg="green")
        self.ch4_label.pack()

        # --- video
        self.video_label = tk.Label(window)
        self.video_label.pack(pady=10)

        # --- joystick knob data
        self._analog_last_lin = 0.0  # last value red by knob
        self._analog_last_ang = 0.0
        self._analog_sent_lin = 0.0  # last value sent to ROS Bridge
        self._analog_sent_ang = 0.0
        self._analog_send_epsilon = 0.02  # only send if the change is above
        self._analog_send_rate_ms = 100  # loop to check (10 Hz)

        # --- joystick frame
        self.joystick_frame = tk.Frame(window)
        self.joystick_frame.pack(pady=10)

        self.create_analog_stick(self.joystick_frame, size=120)

        # --- ROS
        self.ros_data = ROS_Bridge()

        # --- hardware joystick
        try:
            self.hw_joystick = Joystick()
            self.mode_label = tk.Label(self.sensor_frame, text=f"Modo: {self.hw_joystick.mode}", font=("Helvetica", 14), fg="green")
            self.mode_label.pack()
        except Exception as e:
            print("Joystick hardware not available:", e)
            self.hw_joystick = None

        # polling rate (ms) to read the commands by joystick and sync with knob
        self._hw_poll_rate_ms = 30  # ≈33Hz
        self._hw_smoothing_alpha = 0.7
        if self.hw_joystick is not None:
            self.window.after(self._hw_poll_rate_ms, self._poll_hw_joystick)

        #--- setup attributes
        self.update_gui()

    def update_gui(self):
        # --- vídeo
        self.set_frame_data()

        # --- CH4
        self.set_ch4_data()

        # --- CO2
        self.set_co2_data()

        # run again in 100ms
        self.window.after(100, self.update_gui)

    def set_frame_data(self):
        #--setup frame data
        if (self.ros_data.image_subscriber is not None) and (self.ros_data.image_subscriber.get_frame() is not None):
            frame = self.ros_data.image_subscriber.get_frame()
            if frame:
                self.video_label.config(image=frame)
                self.video_label.image = frame

    # --setup ch4 data
    def set_ch4_data(self):
        if (self.ros_data.gas_ch4_subscriber is not None) and (
                self.ros_data.gas_ch4_subscriber.get_ch4_value() is not None):
            ch4_value = self.ros_data.gas_ch4_subscriber.get_ch4_value()
            dangerous_ch4 = 5000
            self.ch4_label.config(text=self.get_gas_text("CH4", dangerous_ch4, ch4_value))

    def set_co2_data(self):
        if (self.ros_data.gas_co2_subscriber is not None) and (
                self.ros_data.gas_co2_subscriber.get_co2_value() is not None):
            co2_value = self.ros_data.gas_co2_subscriber.get_co2_value()
            dangerous_co2 = 1000
            self.co2_label.config(text=self.get_gas_text("CO2", dangerous_co2, co2_value))

    def get_gas_text(self, gas, dangerous_quantity, gas_value):
        return f"🚨Índice de {gas} alto: {gas_value}ppm" if gas_value > dangerous_quantity else f"{gas}: {gas_value} ppm"

    def send_cmd(self, command):
        if self.ros_data.robot_command_publisher is not None:
            self.ros_data.robot_command_publisher.send_commands(command=command)

    # --- closing the app ---
    def on_closing(self):
        # fecha o joystick hardware se existir
        try:
            if self.hw_joystick is not None:
                self.hw_joystick.quit_joystick()
        except Exception:
            pass

        try:
            self.ros_data.on_ros_close()
        except Exception:
            pass

        self.window.destroy()

    #analogic joystick
    def create_analog_stick(self, parent, size=120):
        self.analog_size = size
        self.analog_radius = size // 2
        self.knob_radius = int(size * 0.18)  # knob size
        self.max_travel = self.analog_radius - self.knob_radius - 4

        self.analog_canvas = tk.Canvas(parent, width=size, height=size, bg='lightgray', highlightthickness=0)
        self.analog_canvas.pack()

        cx, cy = self.analog_radius, self.analog_radius

        # external base
        self.analog_canvas.create_oval(
            cx - self.analog_radius + 2, cy - self.analog_radius + 2,
            cx + self.analog_radius - 2, cy + self.analog_radius - 2,
            outline='black', width=2, fill='#ddd'
        )

        # divide the circle in 4
        self.analog_canvas.create_line(cx, 4, cx, size - 4, fill='#bbb')
        self.analog_canvas.create_line(4, cy, size - 4, cy, fill='#bbb')

        # knob initial (in center)
        self.knob = self.analog_canvas.create_oval(
            cx - self.knob_radius, cy - self.knob_radius,
            cx + self.knob_radius, cy + self.knob_radius,
            fill='darkgray', outline='black', width=1
        )

        self.analog_label = tk.Label(parent, text="L:0.00  A:0.00")
        self.analog_label.pack(pady=(6, 0))

        # state
        self._analog_dragging = False
        self._analog_last_lin = 0.0
        self._analog_last_ang = 0.0

        # binds de mouse
        self.analog_canvas.bind("<Button-1>", lambda e: self._analog_press(e))
        self.analog_canvas.bind("<B1-Motion>", lambda e: self._analog_move(e))
        self.analog_canvas.bind("<ButtonRelease-1>", lambda e: self._analog_release(e))

        # loop para envio contínuo enquanto houver movimento (taxa)
        self._analog_send_rate_ms = 100  # 10 Hz por padrão
        self._analog_send_loop()  # inicia loop

    # mouse press
    def _analog_press(self, event):
        self._analog_dragging = True
        self._analog_move(event)

    # mouse move drag
    def _analog_move(self, event):
        # coordinates
        x = event.x
        y = event.y
        cx = self.analog_radius
        cy = self.analog_radius

        # calculate the vector to the center till the point it is selected
        dx = x - cx
        dy = y - cy

        # limit by the circunference
        dist = (dx * dx + dy * dy) ** 0.5
        if dist > self.max_travel:
            scale = self.max_travel / dist
            dx *= scale
            dy *= scale

        # position in knob
        nx = cx + dx
        ny = cy + dy

        # move knob
        self.analog_canvas.coords(
            self.knob,
            nx - self.knob_radius, ny - self.knob_radius,
            nx + self.knob_radius, ny + self.knob_radius
        )

        # normalize
        norm_x = dx / float(self.max_travel)
        norm_y = -dy / float(self.max_travel)  # inverte Y para que "up" seja positivo

        # small deadzone
        deadzone = 0.08
        linear = 0.0 if abs(norm_y) < deadzone else norm_y
        angular = 0.0 if abs(norm_x) < deadzone else norm_x

        # update label
        self.analog_label.config(text=f"L:{linear:.2f}  A:{angular:.2f}")

        # save last values
        self._analog_last_lin = float(linear)
        self._analog_last_ang = float(angular)

    # mouse release -> if drops the moouse, back to the center
    def _analog_release(self, event):
        # return the knob to the center
        cx = self.analog_radius
        cy = self.analog_radius
        self.analog_canvas.coords(
            self.knob,
            cx - self.knob_radius, cy - self.knob_radius,
            cx + self.knob_radius, cy + self.knob_radius
        )
        self.analog_label.config(text="L:0.00  A:0.00")
        self._analog_dragging = False
        self._analog_last_lin = 0.0
        self._analog_last_ang = 0.0

    # loop to send the commands, if the coordinates change
    import time
    def _analog_send_loop(self):
        lin = float(self._analog_last_lin)
        ang = float(self._analog_last_ang)

        send = False

        # absolute difference between values
        d_lin = abs(lin - float(self._analog_sent_lin))
        d_ang = abs(ang - float(self._analog_sent_ang))

        # if change the value in the circle, should send the data
        if d_lin > self._analog_send_epsilon or d_ang > self._analog_send_epsilon:
            send = True

        # if the value backs to the center, should send to stop it
        center_threshold = 0.01
        if abs(lin) < center_threshold and abs(ang) < center_threshold:
            if abs(self._analog_sent_lin) > center_threshold or abs(self._analog_sent_ang) > center_threshold:
                send = True
                lin = 0.0
                ang = 0.0

        if send:
            # check if publisher already existis (its running in another thread)
            if (self.ros_data is not None) and (self.ros_data.robot_command_publisher is not None):
                try:
                    msg = {
                        'linear': {'x': round(float(lin), 2), 'y': 0, 'z': 0},
                        'angular': {'x': 0, 'y': 0, 'z': round(float(ang), 2) * -1}
                    }
                    self.ros_data.robot_command_publisher.send_commands(msg)
                    # update last coordinates that has been sent
                    self._analog_sent_lin = float(lin)
                    self._analog_sent_ang = float(ang)
                except Exception as e:
                    print("Analog send error:", e)

        # reschedule the loop
        self.window.after(self._analog_send_rate_ms, self._analog_send_loop)

    # --- hardware polling methods (integrate joystick with GUI) ---
    def _poll_hw_joystick(self):
        if self.hw_joystick is not None:
            try:
                self.hw_joystick.set_mode(self.mode_label)
                joystick_speed_data = self.hw_joystick.get_angular_vel()
                speed_mode = self.hw_joystick.get_speed_mode()
                raw_ang = joystick_speed_data['x'] * speed_mode #normalize the linear speed by mode
                raw_linear = joystick_speed_data['y'] * speed_mode  #normalize the linear speed by mode
            except Exception as e:
                raw_ang = 0.0
                raw_linear = 0.0
                print(f"ERROR1: {e}")

            #define float angular and linear speed
            target_angular = float(raw_ang)
            target_linear = float(raw_linear)

            # apply deadzone
            deadzone = 0.08
            if abs(target_angular) < deadzone:
                target_angular = 0.0

            # suaviza as transições (alpha 0..1): new = alpha*target + (1-alpha)*current
            a = self._hw_smoothing_alpha
            current_lin = getattr(self, "_analog_last_lin", 0.0)
            current_ang = getattr(self, "_analog_last_ang", 0.0)

            new_lin = a * target_linear + (1 - a) * current_lin
            new_ang = a * target_angular + (1 - a) * current_ang

            # if it's not using knob, go to the center
            if not getattr(self, "_analog_dragging", False):
                self._set_knob_from_normalized(new_lin, new_ang)

        # reschedule
        self.window.after(self._hw_poll_rate_ms, self._poll_hw_joystick)

    def _set_knob_from_normalized(self, linear, angular):
        # cx, cy center
        cx = self.analog_radius
        cy = self.analog_radius

        dx = float(angular) * self.max_travel
        dy = -float(linear) * self.max_travel  # linear 1 -> up -> diminui y

        nx = cx + dx
        ny = cy + dy

        # limit values to inside circle
        dist = ((nx - cx) ** 2 + (ny - cy) ** 2) ** 0.5
        if dist > self.max_travel:
            scale = self.max_travel / dist
            nx = cx + (nx - cx) * scale
            ny = cy + (ny - cy) * scale

        # move knob no canvas
        try:
            self.analog_canvas.coords(
                self.knob,
                nx - self.knob_radius, ny - self.knob_radius,
                nx + self.knob_radius, ny + self.knob_radius
            )
        except Exception:
            pass

        # update label
        self.analog_label.config(text=f"L:{linear:.2f}  A:{angular:.2f}")

        # update last values
        self._analog_last_lin = float(linear)
        self._analog_last_ang = float(angular)


if __name__ == '__main__':
    root = tk.Tk()
    root.geometry("950x910")
    app = GUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
