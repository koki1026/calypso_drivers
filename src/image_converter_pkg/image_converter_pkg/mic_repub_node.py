import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from sensor_msgs.msg import PointCloud2
import numpy as np
from functools import partial

class MicRepubNode(Node):
    """
    Float32MultiArray --> Float32MultiArray
    /velodyne_points を受信している間、/velodyne_points/compressed に1HzでBoolを配信
    """
    def __init__(self):
        super().__init__('mic_repub_node')

        # ===== 既存のマイク関連の初期化 =====
        self.topics = {
            '/mic1/audio': '/mic1/audio/compressed',
            '/mic2/audio': '/mic2/audio/compressed',
            '/mic3/audio': '/mic3/audio/compressed',
            '/mic4/audio': '/mic4/audio/compressed',
            '/mic5/audio': '/mic5/audio/compressed',
            '/mic6/audio': '/mic6/audio/compressed',
            '/mic7/audio': '/mic7/audio/compressed',
            '/mic8/audio': '/mic8/audio/compressed'
        }

        self.subscriptions_ = []
        self.publishers_ = {}

        self.get_logger().info('ノードを初期化しています...')

        for input_topic, output_topic in self.topics.items():
            self.publishers_[input_topic] = self.create_publisher(Float32MultiArray, output_topic, 10)
            callback = partial(self.listener_callback, input_topic=input_topic)
            subscription = self.create_subscription(
                Float32MultiArray,
                input_topic,
                callback,
                10)
            self.subscriptions_.append(subscription)
            self.get_logger().info(f'購読設定完了: "{input_topic}" -> "{output_topic}"')

        # ===== ここからVelodyne関連の機能 =====
        # Velodyne用Publisherの作成
        self.velodyne_publisher_ = self.create_publisher(Bool, '/velodyne_points/compressed', 10)

        # Velodyne用Subscriberの作成

        self.velodyne_subscription_ = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.velodyne_callback,
            10)
        self.get_logger().info('購読設定完了: "/velodyne_points"')

        # 1Hzで配信するためのタイマー
        self.velodyne_publish_timer_ = None
        # サブスクライブが途切れたことを検知するためのタイマー
        self.velodyne_watchdog_timer_ = None
        # タイムアウト時間（秒）
        self.VELODYNE_TIMEOUT_SEC = 2.0
        # ===== Velodyne関連ここまで =====

    def listener_callback(self, msg: Float32MultiArray, input_topic: str):
        """
        メッセージを受信した際のコールバック関数 (既存の機能)
        """
        input_array = np.array(msg.data, dtype=np.float32)
        n = len(input_array)

        if n < 8:
            self.get_logger().warn(
                f'トピック "{input_topic}" で受信した配列の要素数({n})が8未満です。処理をスキップします。')
            return

        # 元の配列を8等分するインデックスを計算
        indices = np.linspace(0, n, num=8, endpoint=False, dtype=int)
        
        # 計算したインデックスを使って、元の配列から値を抽出
        header_values = input_array[indices]

        # 対応するPublisherを取得してメッセージを配信
        publisher = self.publishers_[input_topic]
        output_msg = Float32MultiArray()
        output_msg.data = header_values.tolist()
        publisher.publish(output_msg)


    # ===== ここから変更・追加したメソッド =====
    def velodyne_callback(self, msg: PointCloud2):
        """
        /velodyne_points を受信するたびに呼び出されるコールバック関数。
        """
        # パブリッシュ用のタイマーが作動していなければ、開始する
        if self.velodyne_publish_timer_ is None or self.velodyne_publish_timer_.is_canceled():
            self.get_logger().info(
                '"/velodyne_points" のメッセージを検知。'
                '1Hzで "/velodyne_points/compressed" への配信を開始・再開します。'
            )
            self.velodyne_publish_timer_ = self.create_timer(1.0, self.velodyne_publish_callback)

        # 監視用のタイマーが既に存在すれば、一旦キャンセルしてリセットする
        if self.velodyne_watchdog_timer_ is not None:
            self.velodyne_watchdog_timer_.cancel()

        # 新しく監視用タイマーを設定。指定時間後にタイムアウト用の関数を呼び出す
        self.velodyne_watchdog_timer_ = self.create_timer(
            self.VELODYNE_TIMEOUT_SEC,
            self.velodyne_timeout_callback
        )

    def velodyne_publish_callback(self):
        """
        タイマーによって1秒ごとに呼び出され、Bool型のTrueを配信する。
        """
        msg = Bool()
        msg.data = True
        self.velodyne_publisher_.publish(msg)

    def velodyne_timeout_callback(self):
        """
        /velodyne_points のメッセージが一定時間来なかった場合に呼び出される。
        """
        self.get_logger().info(
            f'"{self.VELODYNE_TIMEOUT_SEC}秒間" /velodyne_points のメッセージがありません。'
            '配信を停止します。'
        )
        # パブリッシュ用のタイマーを停止
        if self.velodyne_publish_timer_ is not None:
            self.velodyne_publish_timer_.cancel()
        # 監視用タイマーも停止（念のため）
        if self.velodyne_watchdog_timer_ is not None:
            self.velodyne_watchdog_timer_.cancel()
    # ===== 変更・追加ここまで =====

def main(args=None):
    rclpy.init(args=args)
    node = MicRepubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
