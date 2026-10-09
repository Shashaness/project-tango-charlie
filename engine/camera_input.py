"""Mouse/trackpad observation input, separate from keyboard vehicle commands."""
import glfw

class CameraInput:
    def __init__(self):
        self.drag = (0.,0.)
        self.scroll = 0.
        self.reset_pressed = False
        self.toggle_pressed = False
        self._last_cursor = None
        self._home_down = False
        self._zero_down = False

    def install(self, window):
        glfw.set_scroll_callback(window, self.on_scroll)

    def on_scroll(self, window, xoffset, yoffset):
        self.scroll += yoffset

    def poll(self, window):
        cursor = glfw.get_cursor_pos(window)
        dragging = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS
        self.drag = (0.,0.)
        if dragging and self._last_cursor is not None:
            self.drag = (cursor[0]-self._last_cursor[0],cursor[1]-self._last_cursor[1])
        self._last_cursor = cursor if dragging else None
        home = any(glfw.get_key(window, key) == glfw.PRESS
                   for key in (glfw.KEY_HOME, glfw.KEY_BACKSPACE))
        self.reset_pressed = home and not self._home_down
        self._home_down = home
        zero = glfw.get_key(window, glfw.KEY_0) == glfw.PRESS
        self.toggle_pressed = zero and not self._zero_down
        self._zero_down = zero

    def apply(self, camera):
        if self.toggle_pressed:
            camera.toggle_external_mode()
            self.drag = (0.,0.)
            self.scroll = 0.
            self._last_cursor = None
        if camera.mode == 'CHASE':
            if self.reset_pressed:camera.reset_orbit()
            else:
                camera.orbit_drag(*self.drag)
                camera.orbit_zoom(self.scroll)
        else:
            self._last_cursor = None
        self.drag = (0.,0.)
        self.scroll = 0.
        self.reset_pressed = False
        self.toggle_pressed = False
