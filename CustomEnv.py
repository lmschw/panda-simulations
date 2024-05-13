from panda_gym.envs.core import RobotTaskEnv
from panda_gym.pybullet import PyBullet

from taskCup import Shake
from CustomPybullet import CustomPybullet

from panda_gym.envs.robots.panda import Panda

class CupEnvironment(RobotTaskEnv):
    """My robot-task environment."""

    def __init__(self, render_mode):
        sim = CustomPybullet(render_mode=render_mode)
        robot = Panda(sim)
        task = Shake(sim)
        super().__init__(robot, task)