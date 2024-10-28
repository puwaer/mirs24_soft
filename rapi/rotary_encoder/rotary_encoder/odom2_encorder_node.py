# dual_encoder_odometry_node.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import pigpio
import tf2_ros
import math

class DualEncoderOdometryNode(Node):
    def __init__(self):
        super().__init__('dual_encoder_odometry_node')

        # エンコーダーのピン番号設定（GPIO番号）
        self.left_clk_pin = 17
        self.left_dt_pin = 18
        self.right_clk_pin = 22
        self.right_dt_pin = 23

        # 車輪のパラメータ
        self.wheel_radius = 0.03  # 車輪半径[m]
        self.wheel_base = 0.2      # 車軸間距離[m]

        # カウント値とエンコーダー設定
        self.left_counter = 0
        self.right_counter = 0
        self.pi = pigpio.pi()  # pigpioのインスタンスを作成
        self.last_left_clk_state = self.pi.read(self.left_clk_pin)
        self.last_right_clk_state = self.pi.read(self.right_clk_pin)

        # ロボットの状態
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # タイムスタンプ
        self.last_time = self.get_clock().now()

        # GPIO割り込み設定
        self.pi.callback(self.left_clk_pin, pigpio.EITHER_EDGE, self.left_rotary_callback)
        self.pi.callback(self.right_clk_pin, pigpio.EITHER_EDGE, self.right_rotary_callback)

        # パブリッシャーとTFブロードキャスターの設定
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # メインループタイマー（50msごとに更新）
        self.create_timer(0.05, self.update_odometry)

        self.get_logger().info("デュアルエンコーダーオドメトリノードを開始します。")

    def left_rotary_callback(self, gpio, level, tick):
        if level != self.last_left_clk_state:
            dt_state = self.pi.read(self.left_dt_pin)
            if dt_state != level:
                self.left_counter += 1
            else:
                self.left_counter -= 1
        self.last_left_clk_state = level

    def right_rotary_callback(self, gpio, level, tick):
        if level != self.last_right_clk_state:
            dt_state = self.pi.read(self.right_dt_pin)
            if dt_state != level:
                self.right_counter += 1
            else:
                self.right_counter -= 1
        self.last_right_clk_state = level

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9  # 経過時間[s]

        # 左右車輪の移動距離を計算
        left_distance = (2.0 * math.pi * self.wheel_radius * self.left_counter) / self.ticks_per_revolution
        right_distance = (2.0 * math.pi * self.wheel_radius * self.right_counter) / self.ticks_per_revolution

        # オドメトリ計算
        delta_distance = (right_distance + left_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_base

        # ロボットの状態を更新
        self.x += delta_distance * math.cos(self.theta)
        self.y += delta_distance * math.sin(self.theta)
        self.theta += delta_theta

        # オドメトリメッセージの作成
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        
        # クォータニオンの計算
        q = self.euler_to_quaternion(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # 速度情報（推定）
        odom_msg.twist.twist.linear.x = delta_distance / dt
        odom_msg.twist.twist.angular.z = delta_theta / dt

        # /odomトピックにパブリッシュ
        self.odom_publisher.publish(odom_msg)

        # TF情報をブロードキャスト
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(transform)

        # 前回の時間を更新
        self.last_time = current_time

        # カウンタをリセット
        self.left_counter = 0
        self.right_counter = 0

    def euler_to_quaternion(self, roll, pitch, yaw):
        """オイラー角からクォータニオンに変換します。"""
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]

    def destroy_node(self):
        self.pi.stop()  # pigpioを停止
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DualEncoderOdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ノードを終了します。')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
