# brackship_controll_ver2

## 概要
BrackShip ROS2-Humble用ドライバ

## インストール
```
$ cd <workspace>
$ git clone #clone this package
$ colcon build
$ source install/setup.bash
```
## 使い方
```
$ ros2 run brackship_controll_ver2 brackship_controll
```
キーボードを使って操縦する
```
ros2 teleop_twist_keybord
```
