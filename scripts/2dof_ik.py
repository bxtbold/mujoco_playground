import mujoco as mj
import numpy as np
import time
from matplotlib import pyplot as plt
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
        t = 0
        dt = 0.01
        num = 250
        x_data = []
        y_data = []

        # start point
        theta1 = np.pi / 3
        theta2 = -np.pi / 2
        self.data.qpos[0] = theta1
        self.data.qpos[1] = theta2

        # create reference
        mj.mj_forward(self.model, self.data)
        sensor_data = self.data.site_xpos[0]
        radius = 0.5
        center = np.array([sensor_data[0] - radius, sensor_data[1]])
        phi = np.linspace(0, 2 * np.pi, num)
        x_ref = center[0] + radius * np.cos(phi)
        y_ref = center[1] + radius * np.sin(phi)

        i = 0
        while not glfw.window_should_close(self.window):
            time_prev = t

            i += 1
            if i > num - 1:
                break

            while (t - time_prev < 1.0 / 60.0):
                # get sensor data
                sensor_data = self.data.site_xpos[0]
                x_sensor_data = sensor_data[0] + np.random.randint(0, 10) * 0.001
                y_sensor_data = sensor_data[1] + np.random.randint(0, 10) * 0.001
                x_data.append(x_sensor_data)
                y_data.append(y_sensor_data)

                # compute Jacobian
                jacp = np.zeros((3, 2))  # 3 is for xyz for two angles
                jacr = None
                mj.mj_jac(self.model, self.data, jacp, jacr, sensor_data, 2)
                J = jacp[[0, 1], :]

                # compute Jacobian IK
                J_inv = np.linalg.inv(J)

                # compute dX
                dX = np.array([x_ref[i] - x_sensor_data, y_ref[i] - y_sensor_data])

                # compute dq = J_inv x dX
                dq = J_inv.dot(dX)

                # update theta1, theta2
                theta1 += dq[0]
                theta2 += dq[1]

                self.data.qpos[0] = theta1
                self.data.qpos[1] = theta2

                mj.mj_forward(self.model, self.data)
                t += dt

            # get framebuffer viewport
            viewport_width, viewport_height = glfw.get_framebuffer_size(self.window)
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

        self.plot_data(x_data, y_data, x_ref, y_ref)

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

    def controller(self, model, data):
        pass

    def set_controller(self):
        mj.set_mjcb_control(self.controller)

    def set_camera_angle(self):
        self.cam.azimuth = 90
        self.cam.elevation = -90
        self.cam.distance = 5.0
        self.cam.lookat = np.array([0, 0, 0])

    def plot_data(self, x_data, y_data, x_ref, y_ref):
        plt.figure(1)
        plt.plot(x_data, y_data, 'bx')
        plt.plot(x_ref, y_ref, 'r-')
        plt.ylabel('y')
        plt.xlabel('x')
        plt.pause(5)
        plt.close()


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
