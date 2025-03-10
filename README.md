# Calypso Drivers

## ROS2 Drivers List

- **✅3D LiDAR driver**
- **Xsens driver**
- **RGB Camera(Grashopper) driver**
- **Thermo Camera(FLIR ONE PRO) driver**
- **MIC array driver**
- **battery level driver**
- **SONOR driver**




# NVIDIA Jetson での Xsens MTi センサーセットアップ

## 1. Xsens SDK のインストール

NVIDIA Jetson で Xsens MTi デバイスを使用するには、Movella のインストールガイドに従ってセットアップを行います。詳細は以下のリンクを参照してください。

[Interfacing MTi devices with the NVIDIA Jetson](https://base.movella.com/s/article/Interfacing-MTi-devices-with-the-NVIDIA-Jetson-1605870420176?language=en_US)

### **注意事項:**
- Jetson は ARM アーキテクチャを採用しているため、ARM 向けのインストール手順を必ず実行してください。
- システムアーキテクチャに合った適切な SDK バージョンをインストールしてください。

---

## 2. Xsens ROS 2 Humble ドライバーのインストール

Xsens MTi デバイスの ROS 2 ドライバーは公式には Humble をサポートしていません。そのため、フォーク版を使用します。

### **リポジトリ詳細:**
- ドライバーは `bluespace-ai` のリポジトリをベースにしています。
- 公式リポジトリでは Humble をサポートしていないため、以下のフォークを使用します。  
  [GitHub Repository - ros2\_0\_humble ブランチ](https://github.com/gmsanchez/bluespace_ai_xsens_ros_mti_driver/tree/ros2_0_humble)

### **インストール手順:**
1. リポジトリをクローンする:
   ```bash
   git clone https://github.com/gmsanchez/bluespace_ai_xsens_ros_mti_driver.git
   ```
2. リポジトリフォルダに移動し、正しいブランチに切り替える:
    ```bash
    cd bluespace_ai_xsens_ros_mti_driver
    git checkout ros2_0_humble
    ```
### **ビルドエラーへの対応:**
colcon build 実行時に以下のエラーが発生する場合:

```bash
/usr/bin/ld: -lxscontroller not found: No such file or directory
/usr/bin/ld: -lxscommon not found: No such file or directory
/usr/bin/ld: -lxstypes not found: No such file or directory
```
以下のコマンドをワークスペース内で実行してください。
    
```bash
pushd src/bluespace_ai_xsens_ros_mti_driver/lib/xspublic && make && popd
```
これにより、リンクに必要な Xsens の公開ライブラリがコンパイルされます。
