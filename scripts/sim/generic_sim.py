import mujoco as mj
import numpy as np
from math import inf
from mujoco.glfw import glfw

class GenericSim(object):

    def __init__(self, xml_path: str, should_visualize=True, **kwargs) -> None:
        self.model = mj.MjModel.from_xml_path(xml_path)
        self.data = mj.MjData(self.model)
        self.should_visualize = should_visualize
        self.sim_time = None
        self.set_controller()
        if self.should_visualize:
            self._init_window(**kwargs)
            self._init_visual()
            self.set_camera_angle()

    def run_simulation(self, dt = 0.01, iter_num = inf):
        self.t = 0
        self.dt = dt
        self.iter_num = iter_num

        self.init_sim_state()

        self.step = 0
        while self._loop_condition():
            time_prev = self.t

            self.step += 1
            if self.step > self.iter_num - 1:
                break

            while self.t - time_prev < 1.0 / 60.0:
                self.control_logic()
                mj.mj_step(self.model, self.data)  # Use mj_step for the full physics step
                self.t += self.dt

            self._render_window()

        self._terminate_window()

    def control_logic(self):
        pass

    def init_sim_state(self):
        pass

    def init_controller(self):
        pass

    def controller(self, model, data):
        pass

    def set_torque_servo(self, actuator_num, kt):
        self.model.actuator_gainprm[actuator_num, 0] = kt

    def set_position_servo(self, actuator_num, kp):
        self.model.actuator_gainprm[actuator_num, 0] = kp
        self.model.actuator_biasprm[actuator_num, 1] = -kp

    def set_velocity_servo(self, actuator_num, kv):
        self.model.actuator_gainprm[actuator_num, 0] = kv
        self.model.actuator_biasprm[actuator_num, 2] = -kv

    def set_controller(self):
        mj.set_mjcb_control(self.controller)

    def set_camera_angle(self, azimuth=90, elevation=-90, distance=5.0, lookat=[0, 0, 0]):
        if self.should_visualize:
            self.cam.azimuth = azimuth
            self.cam.elevation = elevation
            self.cam.distance = distance
            self.cam.lookat = np.array(lookat)

    def _render_window(self):
        if self.should_visualize:
            viewport_width, viewport_height = glfw.get_framebuffer_size(self.window)
            viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
            mj.mjv_updateScene(self.model, self.data, self.option, None, self.cam, mj.mjtCatBit.mjCAT_ALL.value, self.scene)
            mj.mjr_render(viewport, self.scene, self.context)
            glfw.swap_buffers(self.window)
            glfw.poll_events()

    def _terminate_window(self):
        if self.should_visualize:
            glfw.terminate()

    def _loop_condition(self):
        if self.should_visualize:
            return not glfw.window_should_close(self.window)
        if self.sim_time is not None:
            return self.t <= self.sim_time
        return True

    def _init_window(self, **kwargs):
        glfw.init()
        self.window = glfw.create_window(**kwargs)
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)

    def _init_visual(self):
        self.cam = mj.MjvCamera()
        self.option = mj.MjvOption()
        mj.mjv_defaultCamera(self.cam)
        mj.mjv_defaultOption(self.option)
        self.scene = mj.MjvScene(self.model, maxgeom=1000)
        self.context = mj.MjrContext(self.model, mj.mjtFontScale.mjFONTSCALE_150.value)
