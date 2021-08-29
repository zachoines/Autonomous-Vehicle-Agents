import gym

from stable_baselines3 import PPO
from stable_baselines3.common import logger

from mlagents_envs.environment import UnityEnvironment
from gym_unity.envs import UnityToGymWrapper


def main():
    unity_env = UnityEnvironment(None)
    env = UnityToGymWrapper(unity_env, uint8_visual=True)
    logger.configure('./logs')

    model = PPO("CnnPolicy", env, verbose=1)
    act = model.learn(total_timesteps=10000)

    # obs = env.reset()
    # for i in range(1000):
    #     action, _states = model.predict(obs, deterministic=True)
    #     obs, reward, done, info = env.step(action)
    #     env.render()
    #     if done:
    #         obs = env.reset()
    print("Saving model to unity_model.pkl")
    act.save("unity_model.pkl")


if __name__ == '__main__':
    main()
