# raspicat_gamepad_controller

Raspberry Pi Catを家庭用ゲームコントローラ (Logicool Wireless Gamepad F710) を用いて操作するためのパッケージです。  
本パッケージは[zaki0929/raspimouse_game_controller](https://github.com/zaki0929/raspimouse_game_controller)をベースに開発されています。

## 動作環境

以下の環境にて動作確認を行っています。

* Raspberry Pi
  * Raspberry Pi 4 model B
* Ubuntu
  * Ubuntu 18.04
* ROS
  * ROS Melodic Morenia
* ROS Package
  * Raspberry Pi Cat Controller - [rt-net/raspicat_ros](https://github.com/rt-net/raspicat_ros)
  * yocs_velocity_smoother - [yujinrobot/yujin_ocs](https://github.com/yujinrobot/yujin_ocs)
  * joystick_drivers - [ros-drivers/joystick_drivers](https://github.com/ros-drivers/joystick_drivers)

## インストール

[rt-net/raspicat_ros](https://github.com/rt-net/raspicat_ros)をインストールとともにインストールされます。
[README](https://github.com/rt-net/raspicat_ros/blob/melodic-devel/README.md)を参照してROSのセットアップとRaspberry Pi Cat制御用ROSパッケージのインストールを完了させてください。

## 使用方法

以下のコマンドで`raspicat`の`raspicat.launch`と同時に`joy`ノードを起動します。
```
$ roslaunch raspicat_gamepad_controller run_with_base_nodes.launch
```

ジョイスティックが`/dev/input/js0`以外のデバイスで認識されている（例えば`/dev/input/js1`）の場合、
以下のようにして起動するデバイスを指定できます。
```
$ roslaunch raspicat_gamepad_controller run_with_base_nodes.launch dev:=/dev/input/js1
```

## 操作方法

まずはじめに、コントローラのモード切替スイッチを __DirectInput__ (D) modeに切り替え、MODEボタンの横のLEDが点灯していることを確認します。

![](https://rt-net.github.io/images/raspberry-pi-cat/gamepad_front.jpg)

Xボタン（青）を押しながら十字ボタンを押すと上下で移動方向（前後）の指定、左右で旋回方向（左右）の指定ができます。  
Xボタンではなく、Bボタン（赤）を押しながら十字ボタンを押すとなめらかに加減速します。

![](https://rt-net.github.io/images/raspberry-pi-cat/gamepad_top.png)

