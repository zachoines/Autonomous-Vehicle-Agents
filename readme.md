* Installation
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
    