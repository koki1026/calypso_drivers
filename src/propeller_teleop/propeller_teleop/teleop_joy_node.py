#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
import numpy as np

class TeleopJoyNode(Node):
    # ESC制御パラメータ
    ESC_REV = 300      # 最大逆回転
    ESC_NEUTRAL = 400  # 停止
    ESC_FWD = 500      # 最大正回転
    
    def __init__(self):
        super().__init__('teleop_joy_node')
        
        # パブリッシャーの設定 - 2つのプロペラのPWM値を配列として送信
        self.propeller_publisher = self.create_publisher(
            Int32MultiArray, 
            'propeller_pwm', 
            10)

        self.lifecycle_publisher = self.create_publisher(
            Bool, 
            'life_cycle_pwm', 
            1)
        
        # ジョイスティック入力のサブスクライバー設定
        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        
        # タイマーを設定（0.1秒間隔 と 1秒間隔）
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.life_cycle_timer = self.create_timer(1, self.life_cycle_callback)

        # ジョイスティックの状態を保存する変数
        self.left_stick_y = 0.0
        self.right_stick_y = 0.0
        self.joy_received = False
        
        self.get_logger().info('プロペラコントローラーを初期化しました')
    
    def joy_callback(self, msg):
        # Logicoolコントローラーの左右スティックのY軸値を取得
        # 通常、左スティックはY軸(1)、右スティックはY軸(3)
        # ※コントローラーによって軸の割り当てが異なる場合は調整が必要
        try:
            # axes配列から左右スティックのY軸の値を取得
            # 通常は-1.0～1.0の範囲で、上が正、下が負の値
            if len(msg.axes) >= 4:  # 十分な軸があるか確認
                self.left_stick_y = msg.axes[1]   # 左スティックY軸
                self.right_stick_y = msg.axes[4]  # 右スティックY軸
                
                # Logicoolコントローラーの場合、Y軸は下が正のため反転が必要かもしれない
                # 動作確認して必要に応じてコメントアウトを解除
                # self.left_stick_y = -self.left_stick_y
                # self.right_stick_y = -self.right_stick_y
                
                self.joy_received = True
                self.get_logger().debug(f'ジョイスティック入力: 左={self.left_stick_y:.2f}, 右={self.right_stick_y:.2f}')
            else:
                self.get_logger().warn('ジョイスティックの軸データが不足しています')
        except Exception as e:
            self.get_logger().error(f'ジョイスティック入力の処理エラー: {str(e)}')
    
    def map_to_pwm(self, stick_value):
        """スティックの値(-1.0～1.0)をPWM値にマッピング（前後対称）"""
        # デッドゾーンの設定
        deadzone = 0.05
        if abs(stick_value) < deadzone:
            return self.ESC_NEUTRAL

        # 最大オフセット（±いくつNEUTRALからずれるか）
        max_offset = min(self.ESC_FWD - self.ESC_NEUTRAL, self.ESC_NEUTRAL - self.ESC_REV)

        # オフセット値を計算してNEUTRALに足す
        pwm = self.ESC_NEUTRAL + stick_value * max_offset
        return int(round(pwm))
    
    def timer_callback(self):
        """タイマーコールバック - 保存されたジョイスティック入力からPWM値を計算して発行"""
        if not self.joy_received:
            # まだジョイスティック入力を受信していない場合
            self.get_logger().info('ジョイスティック入力を待機中...')
            return
        
        # PWM値に変換
        left_pwm = self.map_to_pwm(self.left_stick_y)
        right_pwm = self.map_to_pwm(self.right_stick_y)
        
        # メッセージの作成
        msg = Int32MultiArray()
        msg.data = [left_pwm, right_pwm]
        
        # メッセージのパブリッシュ
        self.propeller_publisher.publish(msg)
        
        # ログ出力（デバッグ用）
        self.get_logger().debug(f'発行: 左PWM={left_pwm}, 右PWM={right_pwm}')

    def life_cycle_callback(self):
        """ライフサイクルコールバック - 定期的にライフサイクルメッセージを発行"""
        msg = Bool()
        msg.data = True
        self.lifecycle_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopJoyNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('キーボード割り込みによる終了')
    except Exception as e:
        node.get_logger().error(f'エラーが発生しました: {str(e)}')
    finally:
        # 終了時の処理
        # プロペラを停止位置に設定
        msg = Int32MultiArray()
        msg.data = [node.ESC_NEUTRAL, node.ESC_NEUTRAL]
        node.propeller_publisher.publish(msg)
        node.get_logger().info('プロペラを停止位置に設定しました')
        
        # クリーンアップ
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()