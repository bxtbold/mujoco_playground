import numpy as np

from sim.generic_sim import GenericSim
from utils.directory import get_model_dir


class SimulatePendulum(GenericSim):

    def __init__(self, xml_path, should_visualize=True):
        super().__init__(
            xml_path, should_visualize,
            width=1200, height=900, title="Pendulum Simulation", monitor=None, share=None
        )
        self.set_camera_angle(
            azimuth=-90,
            elevation=-2.8,
            distance=5.45,
            lookat=[0, 0, 3]
        )

    def init_sim_state(self):
        self.sim_time = 10
        self.data.qpos[0] = np.pi / 2

    def controller(self, model, data):
        # # position control: spring-like
        # self.set_position_servo(1, 10)
        # self.data.ctrl[1] = np.pi

        # # velocity control
        # self.set_velocity_servo(2, 100)
        # self.data.ctrl[2] = 0.5

        # # position control: pos + vel
        # self.set_position_servo(1, 100)
        # self.set_velocity_servo(2, 10)
        # self.data.ctrl[1] = np.pi

        # torque control: achieve all above
        self.set_torque_servo(0, 1)
        # self.data.ctrl[0] = -10 * (self.data.qpos[0] - np.pi)  # torque (spring)
        # self.data.ctrl[0] = -100 * (self.data.qvel[0] - 0.5)  # speed control
        self.data.ctrl[0] = -100 * (self.data.qpos[0] - np.pi) -10 * self.data.qvel[0]  # position control

    def control_logic(self, step):
        pass  # Control logic handled by the controller callback


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
