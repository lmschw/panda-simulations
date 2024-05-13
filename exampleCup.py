import gymnasium as gym
import panda_gym
import panda_gym.pybullet

from taskCup import Shake

"""
env = gym.make('PandaReach-v3', render_mode="human")

observation, info = env.reset()

for _ in range(1000):
    action = env.action_space.sample() # random action
    observation, reward, terminated, truncated, info = env.step(action)

    if terminated or truncated:
        observation, info = env.reset()
"""
"""
env = gym.make("PandaReach-v3", render_mode="human")
observation, info = env.reset()

for _ in range(1000):
    current_position = observation["observation"][0:3]
    desired_position = observation["desired_goal"][0:3]
    action = 5.0 * (desired_position - current_position)
    observation, reward, terminated, truncated, info = env.step(action)

    if terminated or truncated:
        observation, info = env.reset()

env.close()

"""

"""
from panda_gym.pybullet import PyBullet

sim = PyBullet(render_mode="human")
task = Shake(sim)

task.reset()
print(task.get_obs())
print(task.get_achieved_goal())
print(task.is_success(task.get_achieved_goal(), task.get_goal()))
print(task.compute_reward(task.get_achieved_goal(), task.get_goal()))

"""

from environmentCup import CupEnvironment

env = CupEnvironment(render_mode="human")

observation, info = env.reset()

for _ in range(1000):
    action = env.action_space.sample() # random action
    observation, reward, terminated, truncated, info = env.step(action)

    if terminated or truncated:
        observation, info = env.reset()