import numpy as np
import cv2


class TargetBox:

    def __init__(self):
        self.x_min = None
        self.x_max = None
        self.y_min = None
        self.y_max = None

    def __str__(self):
        return "({}, {}), ({}, {})".format(self.x_min, self.y_min, self.x_max, self.y_max)


class TargetSelector:

    def __init__(self, window: str, scale):
        self.target_box = TargetBox()
        self.state = "INIT"
        self.window = window
        self.scale = scale
        self.cursor_x = None
        self.cursor_y = None
        cv2.setMouseCallback(self.window, self.mouse_handler)

    def convert_to_non_scaled(self, x, y):
        x_converted = int(x / self.scale)
        y_converted = int(y / self.scale)
        return (x_converted, y_converted)

    def mouse_handler(self, event, x, y, flags, param):
        if (event == cv2.EVENT_LBUTTONDOWN):
            self.left_click_handler(x, y)
        elif (event == cv2.EVENT_RBUTTONDOWN):
            self.right_click_handler()
        elif (event == cv2.EVENT_MOUSEMOVE):
            self.mouse_move_handler(x, y)

    def mouse_move_handler(self, x, y):
        self.cursor_x, self.cursor_y = self.convert_to_non_scaled(x, y)

    def left_click_handler(self, _x, _y):
        x, y = self.convert_to_non_scaled(_x, _y)
        if (self.state == "INIT"):
            self.target_box.x_min = x
            self.target_box.x_max = x
            self.target_box.y_min = y
            self.target_box.y_max = y
            self.state = "DRAW"
        elif (self.state == "DRAW"):
            self.target_box.x_min = min(x, self.target_box.x_min)
            self.target_box.y_min = min(y, self.target_box.y_min)
            self.target_box.x_max = max(x, self.target_box.x_max)
            self.target_box.y_max = max(y, self.target_box.y_max)
            self.state = "READY"

    def right_click_handler(self):
        if (self.state != "TRACK"): 
            self.target_box = TargetBox()
            self.cursor_x = None
            self.cursor_y = None
            self.state = "INIT"

    def draw_box(self, frame: cv2.Mat):
        if (self.state == "INIT"):
            return
        elif (self.state == "DRAW"):
            x_min = min(self.cursor_x, self.target_box.x_min)
            y_min = min(self.cursor_y, self.target_box.y_min)
            x_max = max(self.cursor_x, self.target_box.x_max)
            y_max = max(self.cursor_y, self.target_box.y_max)
        else:
            x_min = self.target_box.x_min
            y_min = self.target_box.y_min
            x_max = self.target_box.x_max
            y_max = self.target_box.y_max

        p0 = (x_min, y_min)
        p1 = (x_max, y_max)
        cv2.rectangle(frame, p0, p1, color=(255, 255, 255), thickness=1)

    def get_state(self):
        return self.state

    def set_state(self, state):
        self.state = state

    def get_target_box(self):
        return self.target_box
