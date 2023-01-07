# BARN Planning

* 智能移动机器人课程大作业
* BARN Challenge in ICRA 2022
* 自写代码为rrt_dwa，双向RRT+剪枝平滑，DWA跟随动态局部目标点

## Requirements

安装ROS依赖项

```bash
sudo apt install ros-melodic-sick-tim
sudo apt install ros-melodic-lms1xx
sudo apt install ros-melodic-velodyne-description
sudo apt install ros-melodic-pointgrey-camera-description

# sudo apt install ros-melodic-ros-control ros-melodic-ros-controllers
sudo apt install ros-melodic-robot-localization
sudo apt install ros-melodic-twist-mux
sudo apt install ros-melodic-interactive-marker-twist-server
sudo apt install ros-melodic-move-base
```

## Run Simulations

```
source devel/setup.bash
cd /src/scripts
python2 my_run.py --gui --id xxx
```
* world 99: 
![world_99](https://github.com/Cuzer-explorer/BARN-Planning/blob/master/src/rrt_dwa/99.gif)
* world 199: 
![world_199](https://github.com/Cuzer-explorer/BARN-Planning/blob/master/src/rrt_dwa/199.gif)

## Acknowledgements

Code references

https://github.com/YuxiangCui/autonomous_navigation_project_2022w

[ros_jackal](https://github.com/Daffan/ros_jackal).

[nav-competition-icra2022](https://github.com/Daffan/nav-competition-icra2022).
