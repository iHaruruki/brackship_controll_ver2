# brackship_controll_ver2

## 概要
BrackShip ROS2-Humble用ドライバ

## インストール
```
$ cd ~/ros2_ws/src  #Go to ros workspace
$ git clone https://github.com/iHaruruki/brackship_controll_ver2.git  #clone this package
$ cd ~/ros2_ws
$ colcon build --symlink-install
$ source install/setup.bash
```
## 使い方
```
$ ros2 run brackship_controll_ver2 brackship_controll
```
キーボードを使って操縦する
```
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
DUALSHOCK4（PlayStation 4専用ワイヤレスコントローラー）で操縦する場合<br>
[joy_to_cmdvel](https://github.com/iHaruruki/joy_to_cmdvel.git)
