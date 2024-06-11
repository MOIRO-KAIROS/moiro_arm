# MOIRO MyCobot Package
## Intro
해당 프로젝트는 MyCobot을 ROS 2 Humble 환경에서 제어(MoveIt2)하기 위한 것입니다.

## Installation
### Source Build
```sh
mkdir -p moiro_ws/src
cd moiro_ws
colcon build
```
### Install dependency packages
다음 패키지들이 필요할 수 있습니다.
- ros2_control
- ros2_controller
- moveit
``` sh
sudo apt install -y ros-humble-ros2-control\
  ros-humble-ros2-controller
```
MoveIt2 패키지 설치는 다음 링크를 참고하십시오.
Moveit Humble Documentation(https://moveit.ros.org/install-moveit2/source/)

## Result(MoveIt2 Move Group 이용)
