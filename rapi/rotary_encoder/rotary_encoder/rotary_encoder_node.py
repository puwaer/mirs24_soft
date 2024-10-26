# rotary_encoder_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import RPi.GPIO as GPIO

class RotaryEncoderNode(Node):
    def __init__(self):
        super().__init__('rotary_encoder_node')

        # ピン番号設定
        self.clk_pin = 17
        self.dt_pin = 18

        # エンコーダーのカウント値
        self.counter = 0
        self.last_clk_state = None

        # GPIO設定
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.clk_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.dt_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # 初期状態を記録
        self.last_clk_state = GPIO.input(self.clk_pin)

        # トピックのパブリッシャー設定
        self.publisher_ = self.create_publisher(Int32, 'encoder_count', 10)

        # GPIO割り込み設定
        GPIO.add_event_detect(self.clk_pin, GPIO.BOTH, callback=self.rotary_callback, bouncetime=50)

        self.get_logger().info("ロータリーエンコーダーノードを開始します。")

    def rotary_callback(self, channel):
        clk_state = GPIO.input(self.clk_pin)
        dt_state = GPIO.input(self.dt_pin)

        # CLKピンが立ち下がりした場合にカウントを増減
        if clk_state != self.last_clk_state:
            if dt_state != clk_state:
                self.counter += 1
            else:
                self.counter -= 1

            # カウント値をパブリッシュ
            msg = Int32()
            msg.data = self.counter
            self.publisher_.publish(msg)

            self.get_logger().info(f"Counter: {self.counter}")

        self.last_clk_state = clk_state

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    rotary_encoder_node = RotaryEncoderNode()

    try:
        rclpy.spin(rotary_encoder_node)
    except KeyboardInterrupt:
        rotary_encoder_node.get_logger().info('ノードを終了します。')
    finally:
        rotary_encoder_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

