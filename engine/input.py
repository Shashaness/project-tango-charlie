"""Local-axis flight commands polled once per frame; opposite keys cancel."""

import glfw


class Input:
    def __init__(self):
        self.space_pressed = self.atmosphere_pressed = self.reset_pressed = False
        self.runway_pressed = False
        self._debug_down = set()
        self.dispense_flare = self.dispense_chaff = self.toggle_ecm = False
        self.select_radar = self.select_ir = False
        self.jump_pressed = False
        self.launch_missile = False
        self.select_target = False
        self.fire_gun = False
        self.toggle_sas = self.toggle_hover = False
        self.toggle_configuration = False
        self.vector_command = 0
        self.brake = False
        self.toggle_camera = self.toggle_axes = False
        self._c_down = self._h_down = False
        self.thrust = self.yaw = self.pitch = self.roll = self.lift = self.strafe = 0

    def poll(self, window):
        def axis(positive, negative):
            return (int(glfw.get_key(window, positive) == glfw.PRESS)
                    - int(glfw.get_key(window, negative) == glfw.PRESS))

        if glfw.get_key(window, glfw.KEY_ESCAPE) == glfw.PRESS:
            glfw.set_window_should_close(window, True)
        self.fire_gun = glfw.get_key(window, glfw.KEY_SPACE) == glfw.PRESS
        self.vector_command = axis(glfw.KEY_X, glfw.KEY_Z)
        self.brake = glfw.get_key(window, glfw.KEY_X) == glfw.PRESS
        self.thrust = axis(glfw.KEY_UP, glfw.KEY_DOWN)
        self.yaw = axis(glfw.KEY_Q, glfw.KEY_E)
        self.pitch = axis(glfw.KEY_S, glfw.KEY_W)
        self.roll = axis(glfw.KEY_D, glfw.KEY_A)
        self.lift = axis(glfw.KEY_R, glfw.KEY_F)
        self.strafe = axis(glfw.KEY_RIGHT, glfw.KEY_LEFT)
        c_down = glfw.get_key(window, glfw.KEY_C) == glfw.PRESS
        h_down = glfw.get_key(window, glfw.KEY_H) == glfw.PRESS
        self.toggle_camera = c_down and not self._c_down
        self.toggle_axes = h_down and not self._h_down
        self._c_down, self._h_down = c_down, h_down
        debug_keys = {key for key in (glfw.KEY_F4, glfw.KEY_F1, glfw.KEY_F2, glfw.KEY_R, glfw.KEY_G, glfw.KEY_F3, glfw.KEY_V, glfw.KEY_TAB, glfw.KEY_M, glfw.KEY_L, glfw.KEY_B, glfw.KEY_J, glfw.KEY_1, glfw.KEY_2, glfw.KEY_K)
                      if glfw.get_key(window, key) == glfw.PRESS}
        self.runway_pressed = glfw.KEY_F4 in debug_keys - self._debug_down
        self.space_pressed = glfw.KEY_F1 in debug_keys - self._debug_down
        self.atmosphere_pressed = glfw.KEY_F2 in debug_keys - self._debug_down
        self.reset_pressed = glfw.KEY_R in debug_keys - self._debug_down
        self.toggle_configuration = glfw.KEY_G in debug_keys - self._debug_down
        self.toggle_sas = glfw.KEY_F3 in debug_keys - self._debug_down
        self.toggle_hover = glfw.KEY_V in debug_keys - self._debug_down
        self.select_target = glfw.KEY_TAB in debug_keys - self._debug_down
        self.launch_missile = glfw.KEY_M in debug_keys - self._debug_down
        edges = debug_keys-self._debug_down
        self.jump_pressed = glfw.KEY_K in edges
        self.dispense_flare = glfw.KEY_L in edges
        self.dispense_chaff = glfw.KEY_B in edges
        self.toggle_ecm = glfw.KEY_J in edges
        self.select_radar = glfw.KEY_1 in edges
        self.select_ir = glfw.KEY_2 in edges
        self._debug_down = debug_keys
