teb_local_planner ROS Package
=============================

The teb_local_planner package implements a plugin to the base_local_planner of the 2D navigation stack. 
The underlying method called Timed Elastic Band locally optimizes the robot's trajectory with respect to trajectory execution time, 
separation from obstacles and compliance with kinodynamic constraints at runtime.

Refer to http://wiki.ros.org/teb_local_planner for more information and tutorials.

Build status of the *melodic-devel* branch:
- ROS Buildfarm (Melodic): [![Melodic Status](http://build.ros.org/buildStatus/icon?job=Mdev__teb_local_planner__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__teb_local_planner__ubuntu_bionic_amd64/)


## Citing the Software

*Since a lot of time and effort has gone into the development, please cite at least one of the following publications if you are using the planner for your own research:*

- C. Rösmann, F. Hoffmann and T. Bertram: Integrated online trajectory planning and optimization in distinctive topologies, Robotics and Autonomous Systems, Vol. 88, 2017, pp. 142–153.
- C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann and T. Bertram: Trajectory modification considering dynamic constraints of autonomous robots. Proc. 7th German Conference on Robotics, Germany, Munich, May 2012, pp 74–79.
- C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann and T. Bertram: Efficient trajectory optimization using a sparse model. Proc. IEEE European Conference on Mobile Robots, Spain, Barcelona, Sept. 2013, pp. 138–143.
- C. Rösmann, F. Hoffmann and T. Bertram: Planning of Multiple Robot Trajectories in Distinctive Topologies, Proc. IEEE European Conference on Mobile Robots, UK, Lincoln, Sept. 2015.
- C. Rösmann, F. Hoffmann and T. Bertram: Kinodynamic Trajectory Optimization and Control for Car-Like Robots, IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Vancouver, BC, Canada, Sept. 2017.

<a href="https://www.buymeacoffee.com/croesmann" target="_blank"><img src="https://cdn.buymeacoffee.com/buttons/lato-black.png" alt="Buy Me A Coffee" height="31px" width="132px" ></a>

## Videos

The left of the following videos presents features of the package and shows examples from simulation and real robot situations.
Some spoken explanations are included in the audio track of the video. 
The right one demonstrates features introduced in version 0.2 (supporting car-like robots and costmap conversion). Please watch the left one first.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=e1Bw6JOgHME" target="_blank"><img src="http://img.youtube.com/vi/e1Bw6JOgHME/0.jpg" 
alt="teb_local_planner - An Optimal Trajectory Planner for Mobile Robots" width="240" height="180" border="10" /></a>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=o5wnRCzdUMo" target="_blank"><img src="http://img.youtube.com/vi/o5wnRCzdUMo/0.jpg" 
alt="teb_local_planner - Car-like Robots and Costmap Conversion" width="240" height="180" border="10" /></a>

## License

The *teb_local_planner* package is licensed under the BSD license.
It depends on other ROS packages, which are listed in the package.xml. They are also BSD licensed.

Some third-party dependencies are included that are licensed under different terms:
 - *Eigen*, MPL2 license, http://eigen.tuxfamily.org
 - *libg2o* / *g2o* itself is licensed under BSD, but the enabled *csparse_extension* is licensed under LGPL3+, 
   https://github.com/RainerKuemmerle/g2o. [*CSparse*](http://www.cise.ufl.edu/research/sparse/CSparse/) is included as part of the *SuiteSparse* collection, http://www.suitesparse.com. 
 - *Boost*, Boost Software License, http://www.boost.org

All packages included are distributed in the hope that they will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the licenses for more details.

## Requirements

Install dependencies (listed in the *package.xml* and *CMakeLists.txt* file) using *rosdep*:

    rosdep install teb_local_planner

##modified by Hu Xingyu
修改后的配置说明。

我在teb的基础上增加了带路和slide功能，因此增加了一些配置参数，具体如下：

在teb_local_planner_params.yaml中可以设置以下参数(下面的值是默认值)：

以下是leadHuman的参数
<br> max_dist: 1.5 (robot距离人的最大距离，建议设置为2.0)
<br> use_lead: False (是否允许开启带路功能，设置为True后开启此功能)
<br> gamma: 1.0 (暂时没有使用，这个参数本来是用来计算loss的，但在这一版本中为了保持loss函数的可微性，没有采用)
<br> noise: 0.0 (在与人的距离约束计算时，noise表示了一个可允许的浮动范围)
<br> nearestK: 1 (在计算与global plan点的约束时，这个值表示了需要考虑的点的数量)
<br> max_plan_dist: 0.0 (这个值表示我们计算出来的局部点与global plan点的最大允许距离，在这个距离范围内不会受到惩罚，超过这个值会受到惩罚)
<br> min_obstacle_dist: 0.5 (这个值表示与障碍物的最小距离，小于这个值会受到惩罚)
<br> plan_cost_exponent: 1.0 (计算plan的cost时的指数放大系数)
<br> obstacle_cost_exponent: 1.0 (计算障碍物的cost时的指数放大系数)
<br> soft_distance: 0.1 (这个距离是用来表示当一个local point距离当前robot的距离小于soft_distance时，robot不进行移动，以避免频繁的小范围移动产生抖动，这个值设置过大会导致robot反应迟钝，建议设置在0.1或者0.05)
<br> soft_theta: 0.2618 (这个是角度的一个允许范围，当robot朝向接近人的时候，且距离小于soft_distance时，就可以不让robot移动了，设置的值是15度，这个角度比较合适。)
<br> local_pose_to_human: False (用来开启robot在带路过程中面朝人的功能，开启后，robot会在带路过程中逐渐向人的方向旋转，并在极限位置或者终点调整位姿，朝向人)
<br> local_pose_to_human_full: False(如果开启了full模式，那么robot在全部的移动中都会先朝向人再移动，而上面的模式只是逐渐向人的方向移动。用户可以修改来体会两者的区别，注意full版本开启时，上面的local_pose_to_human最好也开启)
<br> consider_human_pose: False (开启robot在带路过程中尽量处于人的正面的功能，开启后，如果robot发现人的朝向和路径背道而驰，robot会放弃当前的局部目标点，而移动到人的侧面一个用户设置的合适方位)
<br> min_approach_distance: 1.0 (在consider_human_pose开启后，robot尝试移动到人员侧面时的允许最小距离)
<br> best_angle: 1.047197 (在consider_human_pose开启后，robot尝试移动到人员侧面时，它与人的位置和人朝向的夹角，这里默认为PI/3)
<br>以下是slide的参数
<br> slow_scale: 0.3 (在slide模式下，缓慢前进时对最大速度的缩小因子，即max=max*slow_scale)
<br> shift_distance: 0.5 (侧向滑动的最大距离)
<br> move_slow_distance: 4 （缓慢前行的最大距离，在本版本中没有设置local_planner可以自动取消slide模式，所以在最大距离处会停止，直到上层取消slide mode）
<br> shift_end_tolerance: 0.05 (判断侧滑目标抵达的范围)
<br> slide_end_tolerance: 0.1 （判断缓慢前行目标抵达的范围）

<br>HOW TO RUN:
<br>  将本文件下载到本地，覆盖本地工作空间中的teb_local_planner,然后编译即可。
<br>  注意要在teb_local_planner_params.yaml中修改参数，否则它和原本的teb_local_planner没有区别。



