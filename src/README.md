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


# CalypsoのプロペラESCとJetson Orin Nanoを接続する

## PCA9685
プロペラ制御のためにPWMによる制御が必要であるため, I2Cインターフェースを持つ16チャンネルのPWM（パルス幅変調）コントローラーであるPCA9685を用いる.

### 1 Jetson Orin Nano - PCA9685接続
詳細は以下のサイトを参考にすると良い
[KOKENSHAの技術ブログ - Jetson nanoとPCA9685でサーボを動かそうとするときのI2Cエラー対処法！](URL "https://kokensha.xyz/jetson/jetson-nano-pca9685-i2c-error-resolution/")

JetsonとPCA9685の接続は
- VCC
- SDA
- SCL
- GND

の4つを接続する. ここでV+には接続しないことに注意

> [!WARNING]
> V+はプロペラ側への電源供給に使われてしまうため, 大電流が流れてJetsonがこわれます.

また, SDAとSCLはJetsonのピンにいくつか割り当てられているが, 先程のURLのピン図を参考に3番と5番(I2C Bus1)を使用すると良い. 特に今回のコードではI2C Bus1を使用することを明示しているためここに接続する必要がある.

### 2 PCA9685のドライバをインストール
以下をインストールすれば良い
```bash
pip3 install Adafruit_PCA9685 --user
```
以下で接続を確認
```bash
sudo i2cdetect -y -r 1
```
出力としては
```
0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: 70 -- -- -- -- -- -- --
```
のようになっていることが理想だが
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- UU -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: UU -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --   
```
のようになっていてもカーネルが管理しているだけで実際は使えるので問題ない.

### 3 プロペラ - PCA9685接続
ESCにて2つの出力をそれぞれのプロペラに割り当てるため, PCA9685のCH0, CH1をESCに接続する
順番は基本的に V+-GND-PWM になっている

### 実行
```bash
python3 test.py
```
