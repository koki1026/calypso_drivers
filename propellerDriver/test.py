import time
import Adafruit_PCA9685

# PCA9685初期化
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)  # 60Hz

# ESC設定（4096分解能）
ESC_MIN = 250   # 最小スロットル (1000μs)
ESC_MAX = 500   # 最大スロットル (2000μs)
ESC_CHANNELS = [0, 1]  # 接続しているチャンネル

# 最初にESCをアーム
print('Arming ESCs...')
for ch in ESC_CHANNELS:
    pwm.set_pwm(ch, 0, ESC_MIN)
time.sleep(2)  # 2秒待つ（ESCが起動するため）

# ここからゆっくり加速
print('Gradually speeding up...')
for speed in range(ESC_MIN, ESC_MAX + 1, 5):  # 5ずつ増やしていく
    for ch in ESC_CHANNELS:
        pwm.set_pwm(ch, 0, speed)
    time.sleep(0.5)  # 500msごとに更新

print('Max speed reached! Holding...')
time.sleep(3)

# そしてまた停止
print('Stopping motors...')
for ch in ESC_CHANNELS:
    pwm.set_pwm(ch, 0, ESC_MIN)
time.sleep(2)

print('Done.')