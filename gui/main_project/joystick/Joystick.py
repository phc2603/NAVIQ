import pygame

'''
we are using gamesir controller T-4, and the setup is according to it
Buttons index: 
{
    0: "A"
    1: "B":
    2: "X",
    3: "Y",
    4: "LB"
    5: "RB"
    6: "central-L",
    7: "central-R",
    8: "Left analog button",
    9: "Right analog button"    
}

Analog index -> 0
'''

RB_ACELLERATOR_BUTTON_INDEX = 5
LEFT_ANALOG_X_AXIS_BUTTON_INDEX = 0
LEFT_ANALOG_Y_AXIS_BUTTON_INDEX = 1

MODES = {
    "Pesado": {"index_button": 1, "speed": 0.5},
    "Normal": {"index_button": 0, "speed": 1},
    "Turbo": {"index_button": 2, "speed": 2},
    "Turbo maximo": {"index_button": 3, "speed": 3}
}


class Joystick:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            print("No joystick detected")
            raise SystemExit
        self.mode = "Normal"
        self.__joy = pygame.joystick.Joystick(0)
        self.__joy.init()

    #get the coordinate positions in the circle
    def get_angular_vel(self):
        pygame.event.pump()
        if not self.is_speeding():
            return {
                "x": 0,
                "y": 0,
                'debug': "NOT SPEEDING"
            }
        x_axis = self.__joy.get_axis(LEFT_ANALOG_X_AXIS_BUTTON_INDEX)
        y_axis = self.__joy.get_axis(LEFT_ANALOG_Y_AXIS_BUTTON_INDEX)
        return {
            "x": x_axis,
            "y": y_axis * -1
        }

    #return if the speed button is active
    def is_speeding(self):
        pygame.event.pump()
        return self.__joy.get_button(RB_ACELLERATOR_BUTTON_INDEX)

    def set_mode(self, mode_label):
        pygame.event.pump()
        if self.__joy.get_button(0):
            self.mode = "Normal"
        elif self.__joy.get_button(1):
            self.mode = "Pesado"
        elif self.__joy.get_button(2):
            self.mode = "Turbo"
        elif self.__joy.get_button(3):
            self.mode = "Turbo maximo"
        mode_label.config(text=f"Modo: {self.mode}")

    def get_speed_mode(self):
        return MODES[self.mode]['speed']

    def quit_joystick(self):
        self.__joy.quit()
        pygame.quit()

