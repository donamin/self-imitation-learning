# Self-Imitation Learning of Locomotion Movements through Termination Curriculum

This repository contains the source code for [our MIG 2019 paper](https://arxiv.org/abs/1907.11842):
- Amin Babadi, Kourosh Naderi, and Perttu Hämäläinen. 2019. Self-Imitation Learning of Locomotion Movements through Termination Curriculum. In Motion, Interaction and Games (MIG ’19), October 28–30, 2019, Newcastle upon Tyne, United Kingdom. ACM, New York, NY, USA, 7 pages.

# Abstract
Animation and machine learning research have shown great advancements in the past decade, leading to robust and powerful methods for learning complex physically-based animations. However, learning can take hours or days, especially if no reference movement data is available. In this paper, we propose and evaluate a novel combination of techniques for accelerating the learning of stable locomotion movements through self-imitation learning of synthetic animations. First, we produce synthetic and cyclic reference movement using a recent online tree search approach that can discover stable walking gaits in a few minutes. This allows us to use reinforcement learning with Reference State Initialization (RSI) to find a neural network controller for imitating the synthesized reference motion. We further accelerate the learning using a novel curriculum learning approach called Termination Curriculum (TC), that adapts the episode termination threshold over time. The combination of the RSI and TC ensures that simulation budget is not wasted in regions of the state space not visited by the final policy. As a result, our agents can learn locomotion skills in just a few hours on a modest 4-core computer. We demonstrate this by producing locomotion movements for a variety of characters.

# Examples
![Training process for the orc character learning to walk forward](img/training.gif)

The gif above shows how the orc character is trained to walk using our approach (after automatic synthesis of a reference locomotion cycle). More examples can be seen in our [Youtube video](https://youtu.be/dxTZk35Ofyg).

# Code Structure
* ```ode-walker```: The C++ component that is used for producing reference motions using [FDI-MCTS](https://github.com/JooseRajamaeki/TVCG18).
* ```baselines-rl```: The Python component that uses [PPO](https://blog.openai.com/openai-baselines-ppo/) implementation provided by [OpenAI Baselines](https://github.com/openai/baselines) to train controllers using reference motions.

You need to run ```ode-walker\SCA\src\OdeSimpleWalker.cpp``` and ```baselines-rl\baselines\ppo1\run_ode.py``` simultaneously to run the whole pipeline. These two codes are connected to each other by socket programming API provided by [ZeroMQ](https://zeromq.org/).

# Parameters
Here are the main parameters defined in ```ode-walker\SCA\src\OdeSimpleWalker.cpp``` for toggling the switches in the code:
* ```static const bool rigTestMode = false;``` The flag for going into the rig test mode to debug the character configuration settings.
* ```static const bool trainPPO = true;``` The flag for turning the reinforcement learning part on or off. When set to ```false```, the code only runs FDI-MCTS and you do not need to run ```baselnies-rl```.
* ```static const int startStateSamplingForPPOAt = 2 * 60 * 30;``` Determines when to start looking for the reference cycle (default set to 2 minutes of simulation). This is used because the early movements produced using FDI-MCTS are noisy and after a few minutes the noise is reduced significantly.
* ```static const int maxEpLenPPO = 100;``` Maximum episode length when running PPO.
* ```static const bool useNegCostAsTaskReward = false;``` Whether or not to use FDI-MCTS cost function as the task reward (used to prodce Fig. 4 in the paper).
* ```static const bool useImitationReward = true;``` Whether or not to use the imitation reward.
* ```static const bool curriculumLearningPPO = true;``` Whether or not to use termination curriculum (TC), as explained in Section 4.4 of the paper.
* ```static const float curriculumInitialThreshold = 0.75f, curriculumFinalThreshold = 0.5f;``` Initial and final threshold for instantaneous reward, when using termination curriculum (TC). Instantaneous reward is linearly interpolated between these values during PPO training.
* ```static const int maxIterationsPPO = 5000;``` Maximum PPO iterations.
* ```static const int maxEpisodesRender = 5;``` Number of test episodes to render during the PPO training.
* ```static const int maxEpisodesLenRender = 1 * 60 * 30;``` Maximum length of test episodes that are rendered during the PPO training. Note that this overrides ```maxEpLenPPO``` only when testing the policy so one can fairly analyze the performance of the agent during the training.

# Prerequisites
- Microsoft Visual Studio 2015 or above.
- Python 3.5 or above
- Tensorflow
- Matplotlib