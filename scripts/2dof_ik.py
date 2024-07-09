import mujoco as mj
import numpy as np
from matplotlib import pyplot as plt

from sim.generic_sim import GenericSim
from utils.directory import get_model_dir


class Simulate2DOF(GenericSim):

    def __init__(self, xml_path, should_visualize = True):
        super().__init__(
            xml_path, should_visualize,
            width=1200, height=900, title="2DOF_Ik", monitor=None, share=None
        )

    def init_sim_state(self):
        self.x_data = []
        self.y_data = []

        # start point
        self.theta1 = np.pi / 3
        self.theta2 = -np.pi / 2
        self.data.qpos[0] = self.theta1
        self.data.qpos[1] = self.theta2

        # create reference
        mj.mj_forward(self.model, self.data)
        sensor_data = self.data.site_xpos[0]
        radius = 0.5
        center = np.array([sensor_data[0] - radius, sensor_data[1]])
        phi = np.linspace(0, 2 * np.pi, self.iter_num)
        self.x_ref = center[0] + radius * np.cos(phi)
        self.y_ref = center[1] + radius * np.sin(phi)

    def control_logic(self, step=0):
        # get sensor data
        sensor_data = self.data.site_xpos[0]
        # adding random noise to the measured data
        x_sensor_data = sensor_data[0] + np.random.randint(0, 10) * 0.001
        y_sensor_data = sensor_data[1] + np.random.randint(0, 10) * 0.001
        self.x_data.append(x_sensor_data)
        self.y_data.append(y_sensor_data)

        # compute Jacobian
        jacp = np.zeros((3, 2))  # 3 is for xyz for two angles
        jacr = None
        mj.mj_jac(self.model, self.data, jacp, jacr, sensor_data, 2)
        J = jacp[[0, 1], :]

        # compute Jacobian IK
        J_inv = np.linalg.inv(J)

        # compute dX
        dX = np.array([self.x_ref[step] - x_sensor_data, self.y_ref[step] - y_sensor_data])

        # compute dq = J_inv x dX
        dq = J_inv.dot(dX)

        # update theta1, theta2
        self.theta1 += dq[0]
        self.theta2 += dq[1]

        self.data.qpos[0] = self.theta1
        self.data.qpos[1] = self.theta2

    def plot_data(self):
        plt.figure(1)
        plt.plot(self.x_data, self.y_data, 'bx')
        plt.plot(self.x_ref, self.y_ref, 'r-')
        plt.ylabel('y')
        plt.xlabel('x')
        plt.pause(5)
        plt.close()


def main():
    import os
    xml_path = os.path.join(get_model_dir(), "2dof.xml")
    robot = Simulate2DOF(xml_path, should_visualize=True)
    robot.run_simulation()
    robot.plot_data()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)
