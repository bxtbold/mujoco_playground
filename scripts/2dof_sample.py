import mujoco as mj
import numpy as np
import time
from mujoco.glfw import glfw

from utils import get_model_dir


class Simulate2DOF:

    def __init__(self, xml_path):
        self.model = mj.MjModel.from_xml_path(xml_path)
        self.data = mj.MjData(self.model)
        self.init_window()
        self.init_visual()
        self.set_camera_angle()
        self.set_controller()

    def run_simulation(self):
        # set controller
        t = 0
        dt = 0.01
        num = 250
        q0_start = 0
        q1_start = 0
        q0_end = 1.57
        q1_end = 1.57
        q0 = np.linspace(q0_start, q0_end, num)
        q1 = np.linspace(q1_start, q1_end, num)

        self.data.qpos[0] = q0_start
        self.data.qpos[1] = q1_start

        i = 0

        while not glfw.window_should_close(self.window):
            time_prev = t

            i += 1
            if i > num - 1:
                break

            while (t - time_prev < 1.0 / 60.0):
                mj.mj_forward(self.model, self.data)
                if i > len(q0) - 1:
                    break
                self.data.qpos[0] = q0[i]
                self.data.qpos[1] = q1[i]

                t += dt

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

        time.sleep(5)
        glfw.terminate()

    def init_window(self):
        glfw.init()
        self.window = glfw.create_window(1200, 900, "Demo", None, None)
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)

    def init_visual(self):
        self.cam = mj.MjvCamera()  # abstract camera
        self.option = mj.MjvOption()  # set visualization option
        mj.mjv_defaultCamera(self.cam)
        mj.mjv_defaultOption(self.option)
        self.scene = mj.MjvScene(self.model, maxgeom=100)
        self.context = mj.MjrContext(self.model, 150)

    def init_controller(self, model, data):
        pass

    def controller(self, model, data):
        pass

    def set_controller(self):
        mj.set_mjcb_control(self.controller)

    def set_camera_angle(self):
        self.cam.azimuth = 90
        self.cam.elevation = -90
        self.cam.distance = 5.0
        self.cam.lookat = np.array([0, 0, 0])


def main():
    import os
    xml_path = os.path.join(get_model_dir(), "2dof.xml")
    robot = Simulate2DOF(xml_path)
    robot.run_simulation()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)
