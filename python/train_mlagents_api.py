from mlagents_envs.environment import UnityEnvironment
from .Config import config_default as config


if __name__ == "__main__":

    # This is a non-blocking call that only loads the environment.
    env = UnityEnvironment(file_name=config.file_name, seed=config.seed, side_channels=[])
    # Start interacting with the environment.
    env.reset()
    behavior_names = env.behavior_specs.keys()