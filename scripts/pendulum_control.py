import mujoco as mj
import numpy as np
import time
from mujoco.glfw import glfw

from utils import get_model_dir


class SimulatePendulum:

    def __init__(self, xml_path):
        self.model = mj.MjModel.from_xml_path(xml_path)
        self.data = mj.MjData(self.model)
        self.init_window()
        self.init_visual()
        self.set_camera_angle()
        self.set_controller()

    def run_simulation(self):
        sim_time = 10
        self.data.qpos[0] = np.pi/2

        while not glfw.window_should_close(self.window):
            time_prev = self.data.time

            while self.data.time - time_prev < 1.0/60.0:
                mj.mj_step(self.model, self.data)

            if self.data.time > sim_time:
                break

            # get framebuffer viewport
            viewport_width, viewport_height = glfw.get_framebuffer_size(
                self.window)
            viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

            # Update scene and render
            mj.mjv_updateScene(self.model, self.data, self.option, None, self.cam,
                            mj.mjtCatBit.mjCAT_ALL.value, self.scene)
            mj.mjr_render(viewport, self.scene, self.context)

            # swap OpenGL buffers (blocking call due to v-sync)
            glfw.swap_buffers(self.window)

            # process pending GUI events, call GLFW callbacks
            glfw.poll_events()

        time.sleep(1)

        glfw.terminate()

    def set_torque_servo(self, actuator_num, kt):
        self.model.actuator_gainprm[actuator_num, 0] = kt

    def set_position_servo(self, actuator_num, kp):
        self.model.actuator_gainprm[actuator_num, 0] = kp
        self.model.actuator_biasprm[actuator_num, 1] = -kp

    def set_velocity_servo(self, actuator_num, kv):
        self.model.actuator_gainprm[actuator_num, 0] = kv
        self.model.actuator_biasprm[actuator_num, 2] = -kv

    def init_window(self):
        glfw.init()
        self.window = glfw.create_window(1200, 900, "Demo", None, None)
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)

    def init_visual(self):
        self.cam = mj.MjvCamera()     # abstract camera
        self.option = mj.MjvOption()  # visualization option
        mj.mjv_defaultCamera(self.cam)
        mj.mjv_defaultOption(self.option)
        self.scene = mj.MjvScene(self.model, maxgeom=1000)
        self.context = mj.MjrContext(self.model, mj.mjtFontScale.mjFONTSCALE_150.value)

    def init_controller(self, model, data):
        pass

    # def controller(model, data):
    def controller(self, model, data):
        pass
        # # position control: spring-like
        # self.set_position_servo(1, 10)
        # self.data.ctrl[1] = np.pi

        # # velocity control
        # self.set_velocity_servo(2, 100)
        # self.data.ctrl[2] = 0.5

        # # position control: pos + vel
        # self.set_position_servo(1, 100)
        # self.set_velocity_servo(2, 100)
        # self.data.ctrl[1] = np.pi

        # torque control: achieve all above
        self.set_torque_servo(0, 1)
        # self.data.ctrl[0] = -10 * (self.data.qpos[0] - np.pi)  # torque (spring)
        # self.data.ctrl[0] = -100 * (self.data.qvel[0] - 0.5)  # speed control
        self.data.ctrl[0] = -100 * (self.data.qpos[0] - np.pi) -10 * self.data.qvel[0]  # position control

    def set_controller(self):
        mj.set_mjcb_control(self.controller)

    def set_camera_angle(self):
        self.cam.azimuth = -90
        self.cam.elevation = -2.8
        self.cam.distance = 5.45
        self.cam.lookat = np.array([0, 0, 3.0])


def main():
    import os
    xml_path = os.path.join(get_model_dir(), "pendulum.xml")
    robot = SimulatePendulum(xml_path)
    robot.run_simulation()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)
