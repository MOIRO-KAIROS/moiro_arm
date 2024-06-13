# MOIRO ARM
## 0. Intro
- ROS2 Humble Packages for MOIRO ARM
- 해당 프로젝트는 Moiro에서 사용된 MyCobot을 ROS 2 Humble 환경에서 제어(MoveIt2)하기 위한 것입니다.

## 1. Installation
### 1) Source Build
```sh
mkdir -p moiro_ws/src
cd moiro_ws
colcon build
```
### 2) Install dependency packages
다음 패키지들이 필요할 수 있습니다.

- ros-humble-gazebo-ros2-control
- ros-humble-ros2-controllers
- ros-humble-controller-manager
- ros-humble-moveit
- (ros-humble-gazebo-ros2-control)

``` sh
sudo apt install -y ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-moveit
```
``` sh
pip install pymycobot moveit_configs_utils
```

#### 참고: MoveIt2 패키지 설치는 다음 링크를 참고하십시오.
[Moveit Humble Documentation](https://moveit.ros.org/install-moveit2/source/)

## 2. Result
- MoveIt2 Move Group 이용

<div align="center">
  <a href="https://youtu.be/h4u68pizy8g">
    <img src="http://img.youtube.com/vi/h4u68pizy8g/0.jpg" alt="Video Label">
  </a>
</div>

```sh
ros2 launch moiro_arm_move_group moiro_arm_move_group.launch.py
ros2 launch moiro_arm_move_group moiro_arm_move_group_topic_interface.launch.py
```

- 실물 MyCobot 제어를 위한 Sync Play
```sh
sudo chmod 777 /dev/ttyACM0
ros2 run moiro_arm_robot_driver sync_play
```
