Simulation bundle  of the paper

R. Der and G. Martius. Novel plasticity rule can explain the development of sensorimotor intelligence. Proceedings of the National Academy of Sciences, 112(45):E6224--E6232, 2015. arXiv Preprint http://arxiv.org/abs/1505.00835.

Check out the supplementary:
http://playfulmachines.com/DEP

Requirements:
- you need Linux (if not then you can use a Virtual Machine like virtual box and install a Ubuntu there)
- you need to install LpzRobots from http://robot.informatik.uni-leipzig.de/software/ you need version 0.81 at least.
  - I recommend to use the git repository on https://github.com/georgmartius/lpzrobots
  because the packages may be a bit outdated
  - first check the Dependencies file in the root of the lpzrobots folder and install the required packages
  - then call make and follow the instructions (do not put sudo in front) If you install to a system wide location it will automatically sudo for the installation.
  - if unsure use the default settings


- onces lpzrobots is install call make in either of the sub-folders here (humanoid/hexapod_simulation)
- follow the instructions in the README.txt files there.



The controller itself is implemented in dep.h and dep.cpp. The core is in the stepNoLearning() function.

Good luck and have fun!
