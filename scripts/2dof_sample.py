import mujoco as mj
import numpy as np

from sim.generic_sim import GenericSim
from utils.directory import get_model_dir


class Simulate2DOF(GenericSim):

    def __init__(self, xml_path, should_visualize = True):
        super().__init__(
            xml_path, should_visualize,
            width=1200, height=900, title="2DOF_Ik", monitor=None, share=None
        )

    def init_sim_state(self):
        # set controller
        self.q0_start = 0
        self.q1_start = 0
        self.q0_end = 1.57
        self.q1_end = 1.57
        self.q0 = np.linspace(self.q0_start, self.q0_end, self.num)
        self.q1 = np.linspace(self.q1_start, self.q1_end, self.num)
        self.data.qpos[0] = self.q0_start
        self.data.qpos[1] = self.q1_start

    def control_logic(self, step):
        mj.mj_forward(self.model, self.data)
        self.data.qpos[0] = self.q0[step]
        self.data.qpos[1] = self.q1[step]


def main():
    import os
    xml_path = os.path.join(get_model_dir(), "2dof.xml")
    robot = Simulate2DOF(xml_path, should_visualize=True)
    robot.run_simulation()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)
