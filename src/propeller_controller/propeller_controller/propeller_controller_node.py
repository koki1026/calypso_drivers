#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import time
import Adafruit_PCA9685

class PropellerControllerNode(Node):
    # ESC設定（4096分解能）
    ESC_REV = 250      # 最大逆回転
    ESC_NEUTRAL = 400  # 停止
    ESC_FWD = 500      # 最大正回転
    ESC_CHANNELS = [0, 1]  # 接続しているチャンネル（左右のプロペラ）
    
    def __init__(self):
        super().__init__('propeller_controller_node')
        
        # PCA9685の初期化
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)  # 60Hz
        
        # サブスクライバーの設定
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'propeller_pwm',
            self.pwm_callback,
            10)
        
        # 現在のPWM値を保存する変数
        self.current_pwm = [self.ESC_NEUTRAL, self.ESC_NEUTRAL]
        
        # ESCのアーム処理
        self.arm_escs()
        
        self.get_logger().info('プロペラドライバーを初期化しました')
    
    def arm_escs(self):
        """ESCをアームする処理"""
        self.get_logger().info('ESCをアーム中...')
        
        # すべてのESCチャンネルに停止信号を送信
        for ch in self.ESC_CHANNELS:
            self.pwm.set_pwm(ch, 0, self.ESC_NEUTRAL)
            
        # ESCが起動するための待機時間
        time.sleep(2)
        self.get_logger().info('ESCのアーム完了')
    
    def pwm_callback(self, msg):
        """PWM値を受信したときのコールバック関数"""
        try:
            if len(msg.data) >= 2:
                # 受信したPWM値を取得
                left_pwm = msg.data[0]
                right_pwm = msg.data[1]
                
                # 値の安全チェック
                left_pwm = self.validate_pwm(left_pwm)
                right_pwm = self.validate_pwm(right_pwm)
                
                # PWM値の大きな変化を制限（急激な変化を防止）
                left_pwm = self.limit_pwm_change(left_pwm, self.current_pwm[0])
                right_pwm = self.limit_pwm_change(right_pwm, self.current_pwm[1])
                
                # PWM値を設定
                self.pwm.set_pwm(self.ESC_CHANNELS[0], 0, left_pwm)
                self.pwm.set_pwm(self.ESC_CHANNELS[1], 0, right_pwm)
                
                # 現在の値を更新
                self.current_pwm = [left_pwm, right_pwm]
                
                self.get_logger().info(f'PWM設定: 左={left_pwm}, 右={right_pwm}')
            else:
                self.get_logger().warn('受信したPWMデータが不足しています')
        except Exception as e:
            self.get_logger().error(f'PWM設定エラー: {str(e)}')
            # エラー時は安全のため停止
            self.emergency_stop()
    
    def validate_pwm(self, pwm_value):
        """PWM値が有効範囲内かチェックし、範囲内に収める"""
        if pwm_value < self.ESC_REV:
            self.get_logger().warn(f'PWM値が最小値を下回っています: {pwm_value} -> {self.ESC_REV}')
            return self.ESC_REV
        elif pwm_value > self.ESC_FWD:
            self.get_logger().warn(f'PWM値が最大値を超えています: {pwm_value} -> {self.ESC_FWD}')
            return self.ESC_FWD
        return pwm_value
    
    def limit_pwm_change(self, new_pwm, current_pwm):
        """PWM値の急激な変化を制限する"""
        # 1回の更新で許容する最大変化量
        max_change = 20
        
        if abs(new_pwm - current_pwm) > max_change:
            if new_pwm > current_pwm:
                return current_pwm + max_change
            else:
                return current_pwm - max_change
        
        return new_pwm
    
    def emergency_stop(self):
        """緊急停止処理"""
        self.get_logger().warn('緊急停止を実行します')
        for ch in self.ESC_CHANNELS:
            self.pwm.set_pwm(ch, 0, self.ESC_NEUTRAL)
        self.current_pwm = [self.ESC_NEUTRAL, self.ESC_NEUTRAL]
    
    def shutdown(self):
        """終了処理"""
        self.get_logger().info('モーターを停止します...')
        for ch in self.ESC_CHANNELS:
            self.pwm.set_pwm(ch, 0, self.ESC_NEUTRAL)
        time.sleep(1)
        self.get_logger().info('プロペラドライバーを終了します')

def main(args=None):
    rclpy.init(args=args)
    driver = PropellerControllerNode()
    
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        driver.get_logger().info('キーボード割り込みによる終了')
    except Exception as e:
        driver.get_logger().error(f'エラーが発生しました: {str(e)}')
    finally:
        # 終了処理
        driver.shutdown()
        driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()