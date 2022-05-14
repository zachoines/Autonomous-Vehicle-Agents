# Autonomous vehicle Agents

AI Navigation with Mecanum Robots. Trained with Unity's MLAgents API.

## Overview 

The purpose of this project is to design and train a reinforcement learning agent to navigate a simulated mecanum robot around obstacles and towards a goal. The information given to the agent are lidar scan data, velocity, encoder positions, and distance/angle to goal. To solve the environment, the agent must build an internal representation as it explores. This is similar conceptually to Simultaneous Localization and Mapping (SLAM) problems, except without the use of external map generation. Instead, this research gauges the effectiveness of recurrent, imitation, and curiosity networks in accomplishing efficient navigation.

[![Demo](https://img.youtube.com/vi/TncZfuzbYik/maxresdefault.jpg)](https://youtu.be/TncZfuzbYik "Demo")

[![Training and Testing](https://img.youtube.com/vi/Qgr-Nf0ts9M/maxresdefault.jpg)](https://youtu.be/Qgr-Nf0ts9M "Training and Testing")

## Installation
    * Python modules
        - pip3 install mlagents mlagents_envs gym_unity
        - pip3 install torch~=1.7.1 -f https://download.pytorch.org/whl/torch_stable.html
        - OpenAI-baslines
    * Import Assets
        * (/Autonomous Vehicle Agents/Assets/robot.blend)[https://drive.google.com/drive/folders/18gfgXw4FeOUl0M7MnyoCyLha8bqXCzx2?usp=sharing]
    * Open Unity Hub
        - Install unity version 2021.1.18 or higher
        - Import "Autonomous Vehicle Agents" project from disk
        - Clone MLAgents on disk
            - git clone --branch release_18 https://github.com/Unity-Technologies/ml-agents.git
            - Copy ml-agents/Project/Assets/ML-Agents to { your project name }/assets/ML-Agents
        - Go to package manager to install ml-agents
            - Install the com.unity.ml-agents Unity package (import package.json from folder of same name)
            - Install the com.unity.ml-agents.extensions Unity package
    * mlagents command line
        * mlagents-learn
    