#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <wiringPi.h>

// エンコーダ関連のピン設定
#define LEFT_ENCODER_PIN_A 17
#define LEFT_ENCODER_PIN_B 18
#define RIGHT_ENCODER_PIN_A 22
#define RIGHT_ENCODER_PIN_B 23

// ロボットのパラメータ
#define WHEEL_RADIUS 0.05  // タイヤの半径(m)
#define WHEEL_BASE 0.25    // 車輪間距離(m)
#define ENCODER_RESOLUTION 360 // エンコーダの分解能(パルス/回転)

using namespace std::chrono_literals;

class OdometryPublisher : public rclcpp::Node
{
public:
    OdometryPublisher()
    : Node("odometry_publisher"), 
      left_encoder_count_(0), 
      right_encoder_count_(0),
      prev_left_encoder_(0),
      prev_right_encoder_(0),
      x_(0.0),
      y_(0.0),
      theta_(0.0)
    {
        // Odometryのパブリッシャーを設定
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        
        // TFブロードキャスターの初期化
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // デバッグ用のパブリッシャー
        debug_pub_ = this->create_publisher<std_msgs::msg::String>("debug_encoder", 10);

        // エンコーダピンの設定
        if (wiringPiSetupGpio() == -1) {
            RCLCPP_ERROR(this->get_logger(), "GPIO初期化エラー");
            return;
        }

        pinMode(LEFT_ENCODER_PIN_A, INPUT);
        pinMode(LEFT_ENCODER_PIN_B, INPUT);
        pinMode(RIGHT_ENCODER_PIN_A, INPUT);
        pinMode(RIGHT_ENCODER_PIN_B, INPUT);

        // プルアップ抵抗を有効化
        pullUpDnControl(LEFT_ENCODER_PIN_A, PUD_UP);
        pullUpDnControl(LEFT_ENCODER_PIN_B, PUD_UP);
        pullUpDnControl(RIGHT_ENCODER_PIN_A, PUD_UP);
        pullUpDnControl(RIGHT_ENCODER_PIN_B, PUD_UP);

        // 割り込みの設定
        if (wiringPiISR(LEFT_ENCODER_PIN_A, INT_EDGE_BOTH, &leftEncoderISR) < 0) {
            RCLCPP_ERROR(this->get_logger(), "左エンコーダの割り込み設定エラー");
        }
        if (wiringPiISR(RIGHT_ENCODER_PIN_A, INT_EDGE_BOTH, &rightEncoderISR) < 0) {
            RCLCPP_ERROR(this->get_logger(), "右エンコーダの割り込み設定エラー");
        }

        // タイマーコールバック (10Hz)
        timer_ = this->create_wall_timer(100ms, std::bind(&OdometryPublisher::updateOdometry, this));
        
        RCLCPP_INFO(this->get_logger(), "Odometryパブリッシャーを初期化しました");
    }

private:
    void updateOdometry()
    {
        auto current_time = this->get_clock()->now();
        
        // エンコーダの差分を計算
        int delta_left = left_encoder_count_ - prev_left_encoder_;
        int delta_right = right_encoder_count_ - prev_right_encoder_;
        
        // エンコーダ値を更新
        prev_left_encoder_ = left_encoder_count_;
        prev_right_encoder_ = right_encoder_count_;

        // 車輪の回転角度（ラジアン）を計算
        double delta_left_rad = (delta_left / static_cast<double>(ENCODER_RESOLUTION)) * 2.0 * M_PI;
        double delta_right_rad = (delta_right / static_cast<double>(ENCODER_RESOLUTION)) * 2.0 * M_PI;

        // それぞれの車輪の移動距離を計算
        double left_distance = delta_left_rad * WHEEL_RADIUS;
        double right_distance = delta_right_rad * WHEEL_RADIUS;

        // ロボットの移動距離と回転量を計算
        double delta_distance = (left_distance + right_distance) / 2.0;
        double delta_theta = (right_distance - left_distance) / WHEEL_BASE;

        // ロボットの位置と姿勢を更新
        x_ += delta_distance * cos(theta_);
        y_ += delta_distance * sin(theta_);
        theta_ += delta_theta;

        // デバッグ情報の出力
        auto debug_msg = std_msgs::msg::String();
        debug_msg.data = "Left: " + std::to_string(left_encoder_count_) + 
                        ", Right: " + std::to_string(right_encoder_count_);
        debug_pub_->publish(debug_msg);

        // Odometryメッセージの作成
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        // 位置の設定
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        // 四元数の計算と設定
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        // 速度の計算と設定（100msごとの更新）
        odom.twist.twist.linear.x = delta_distance / 0.1;  // 0.1秒周期なので0.1で割る
        odom.twist.twist.angular.z = delta_theta / 0.1;

        // パブリッシュ
        odom_pub_->publish(odom);

        // TF変換のパブリッシュ
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = current_time;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";
        
        transform.transform.translation.x = x_;
        transform.transform.translation.y = y_;
        transform.transform.translation.z = 0.0;
        
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(transform);
    }

    // 左エンコーダの割り込み処理
    static void leftEncoderISR()
    {
        static int last_state = 0;
        int state = digitalRead(LEFT_ENCODER_PIN_A);
        int b_state = digitalRead(LEFT_ENCODER_PIN_B);
        
        if (state != last_state) {
            if (b_state != state) {
                left_encoder_count_++;
            } else {
                left_encoder_count_--;
            }
        }
        last_state = state;
    }

    // 右エンコーダの割り込み処理
    static void rightEncoderISR()
    {
        static int last_state = 0;
        int state = digitalRead(RIGHT_ENCODER_PIN_A);
        int b_state = digitalRead(RIGHT_ENCODER_PIN_B);
        
        if (state != last_state) {
            if (b_state != state) {
                right_encoder_count_++;
            } else {
                right_encoder_count_--;
            }
        }
        last_state = state;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    static volatile int left_encoder_count_;
    static volatile int right_encoder_count_;
    int prev_left_encoder_;
    int prev_right_encoder_;
    
    // ロボットの位置と向き
    double x_;
    double y_;
    double theta_;
};

// static変数の初期化
volatile int OdometryPublisher::left_encoder_count_ = 0;
volatile int OdometryPublisher::right_encoder_count_ = 0;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryPublisher>());
    rclcpp::shutdown();
    return 0;
}