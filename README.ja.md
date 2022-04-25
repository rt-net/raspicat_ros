[![industrial_ci](https://github.com/rt-net/raspicat_ros/workflows/industrial_ci/badge.svg?branch=master)](https://github.com/rt-net/raspicat_ros/actions?query=workflow%3Aindustrial_ci+branch%3Amaster)

# raspicat

## 必要環境

下記の環境にて動作確認を行っています。

* Raspberry Pi
  * Raspberry Pi 4 model B
* Ubuntu
  * Ubuntu 18.04
* ROS 
  * ROS Melodic Morenia
* Device Driver
  * [rt-net/RaspberryPiMouse](https://github.com/rt-net/RaspberryPiMouse)

## インストール

### 1. 最新版のROSをインストール  

[ROS WiKi](http://wiki.ros.org/melodic/Installation)を参照するか、以下のスクリプトを使用してROSのインストールを完了させてください。

* [ryuichiueda/ros_setup_scripts_Ubuntu18.04_server](https://github.com/ryuichiueda/ros_setup_scripts_Ubuntu18.04_server)

### 2. デバイスドライバのダウンロードとインストール  

[rt-net/RaspberryPiMouse](https://github.com/rt-net/RaspberryPiMouse)のREADMEを参照してください。

### 3. ワークスペースの作成

ROSインストール後、以下のようにワークスペースを作成します。

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_worksapce
cd ~/catkin_ws
catkin_make
```

以下の行を `~/.bashrc` から削除します。

```
source /opt/ros/melodic/setup.bash
```

以下の行を `~/.bashrc` に追加します。

```
source ~/catkin_ws/devel/setup.bash
```

### 4. 本リポジトリを `~/catkin_ws/src` にダウンロード

以下のコマンドで本リポジトリを先ほど作成したワークスペースにダウンロードします。

```
cd ~/catkin_ws/src
git clone https://github.com/rt-net/raspicat_ros.git
```

### 5. 依存関係にあるパッケージのインストール

以下のコマンドで依存関係にあるパッケージをまとめてインストールします。

```
cd ~/catkin_ws
rosdep install -r -y --from-paths src
```

### 6. 本パッケージのビルド

以下のコマンドで本リポジトリをビルドします。
ここでエラーが出てしまった場合はステップ5を再度実行してみてください。
それでも解決しない場合は[Issues](https://github.com/rt-net/raspicat_ros/issues)にてお問い合わせください。

```
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```


### 7. ブザーのテスト

以下のコマンドを実行し、基盤となるプログラム群を起動します。

```
roslaunch raspicat raspicat.launch
```

次に別の端末にて以下のコマンドを実行します。
Raspberry Pi Catに搭載されているブザーが鳴ります。

```
rostopic pub /buzzer std_msgs/UInt16 1000
```

## ライセンス
このリポジトリはApache 2.0ライセンスの元、公開されています。
ライセンスについては[LICENSE](./LICENSE)を参照ください。
