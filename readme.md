* Installation
    * Install unity version 2020.20.3 or higher
    * Clone MLAgents
        - git clone --branch release_18 https://github.com/Unity-Technologies/ml-agents.git
    * Python modules
        - mlagents mlagents_envs gym_unity
        - pip3 install torch~=1.7.1 -f https://download.pytorch.org/whl/torch_stable.html
        - OpenAI-baslines
            - 
    * Open unity
        - create new project
        - go to package manager to install ml-agents
            - Install the com.unity.ml-agents Unity package (import package.json from folder of same name)
            - Install the com.unity.ml-agents.extensions Unity package
        - Copy ml-agents/Project/Assets/ML-Agents to { your project name }/assets/ML-Agents
    * mlagents command line
        * mlagents-learn
    