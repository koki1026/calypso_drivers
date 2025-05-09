import time
import Adafruit_PCA9685

# PCA9685初期化
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)  # 60Hz

# ESC設定（4096分解能）
ESC_REV = 300    # 最大逆回転
ESC_NEUTRAL = 366  # 停止
ESC_FWD = 400    # 最大正回転
ESC_CHANNELS = [0, 1]  # 接続しているチャンネル

# 最初にESCをアーム
print('Arming ESCs...')
for ch in ESC_CHANNELS:
    pwm.set_pwm(ch, 0, ESC_NEUTRAL)
time.sleep(2)  # 2秒待つ（ESCが起動するため）

# ここからゆっくり加速
for speed in range(ESC_NEUTRAL, ESC_FWD + 1, 20):
    for ch in ESC_CHANNELS:
        pwm.set_pwm(ch, 0, speed)
    print(f"Speed step: {speed}")  # <- リアルタイム表示
    time.sleep(0.5)

print("Starting reverse slowly...")
for speed in range(ESC_NEUTRAL, ESC_REV - 1, -20):
    for ch in ESC_CHANNELS:
        pwm.set_pwm(ch, 0, speed)
    print(f"Speed step: {speed}")  # <- リアルタイム表示
    time.sleep(0.5)

# 最後に停止
print('Stopping motors...')
for ch in ESC_CHANNELS:
    pwm.set_pwm(ch, 0, ESC_NEUTRAL)
time.sleep(2)

print('Done.')