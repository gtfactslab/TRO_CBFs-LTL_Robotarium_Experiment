# Comparison between traditional finite time control barrier functions and composite finite time control barrier functions

The objective of the code in this folder is to showcase the advantage of Theorem 1 in the paper. The example consists of two agents, and two regions of interest A and B. Agent 1 must visit region A, and agent 2 must visit region B. In addition, both the agents must obey a connectivity constraint which is a function of the state of agent 2.

By following the traditional approach of encoding each finite time barrier function separately in a quadratic program, we observe that the optimization program is infeasible, and hence, the problem is infeasible. This is shown below, where the trajectories stop at the instant of infeasibility.

![Alt text](/figs/trad_traj.png?raw=true)

However, by using Theorem 1 in our paper, one can relax this requirement and obtain a feasible solution as is shown in the figure below:

![Alt text](/figs/comp_traj.png?raw=true)

