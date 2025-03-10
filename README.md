# Calypso Drivers

## ROS2 Drivers List

- **✅3D LiDAR driver**
- **Xsens driver**
- **RGB Camera(Grashopper) driver**
- **Thermo Camera(FLIR ONE PRO) driver**
- **MIC array driver**
- **battery level driver**
- **SONOR driver**




NVIDIA Jetson での Xsens MTi センサーセットアップ

1. Xsens SDK のインストール

NVIDIA Jetson で Xsens MTi デバイスを使用するには、Movella のインストールガイドに従ってセットアップを行います。詳細は以下のリンクを参照してください。

Interfacing MTi devices with the NVIDIA Jetson

注意事項:

Jetson は ARM アーキテクチャを採用しているため、ARM 向けのインストール手順を必ず実行してください。

システムアーキテクチャに合った適切な SDK バージョンをインストールしてください。

2. Xsens ROS 2 Humble ドライバーのインストール

Xsens MTi デバイスの ROS 2 ドライバーは公式には Humble をサポートしていません。そのため、フォーク版を使用します。

リポジトリ詳細:

ドライバーは bluespace-ai のリポジトリをベースにしています。

公式リポジトリでは Humble をサポートしていないため、以下のフォークを使用します。
GitHub Repository - ros2_0_humble ブランチ

インストール手順:

リポジトリをクローンする:

git clone https://github.com/gmsanchez/bluespace_ai_xsens_ros_mti_driver.git

リポジトリフォルダに移動し、正しいブランチに切り替える:

cd bluespace_ai_xsens_ros_mti_driver
git checkout ros2_0_humble

ROS 2 ワークスペース内の src フォルダに移動:

mv bluespace_ai_xsens_ros_mti_driver ~/ros2_ws/src/

colcon を使用してビルド:

cd ~/ros2_ws
colcon build

ビルドエラーへの対応:

colcon build 実行時に以下のエラーが発生する場合:

/usr/bin/ld: -lxscontroller not found: No such file or directory
/usr/bin/ld: -lxscommon not found: No such file or directory
/usr/bin/ld: -lxstypes not found: No such file or directory

以下のコマンドを ROS 2 ワークスペース内で実行してください。

pushd src/bluespace_ai_xsens_ros_mti_driver/lib/xspublic && make && popd

これにより、リンクに必要な Xsens の公開ライブラリがコンパイルされます。

3. インストールの確認

インストール後、以下のコマンドを実行してドライバーが正しく動作するか確認します。

ros2 run xsens_driver mtnode

正しくセットアップされていれば、MTi デバイスが ROS 2 上でセンサーデータを出力します。

