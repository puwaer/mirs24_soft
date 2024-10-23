#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <wiringPi.h>

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
        // パラメータの宣言
        this->declare_parameter("wheel_radius", 0.05);
        this->declare_parameter("wheel_base", 0.25);
        this->declare_parameter("encoder_resolution", 360);

        // パラメータの取得
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        encoder_resolution_ = this->get_parameter("encoder_resolution").as_int();

        // Odometryのパブリッシャーを設定
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        
        // TFブロードキャスターの初期化
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // デバッグ用のパブリッシャー
        debug_pub_ = this->create_publisher<std_msgs::msg::String>("debug_encoder", 10);

        // エンコーダピンの設定
        if (wiringPiSetupGpio() == -1) {
            RCLCPP_ERROR(this->get_logger(), "GPIO initialization error");
            return;
        }

        setupEncoderPins();

        // タイマーコールバック (10Hz)
        timer_ = this->create_wall_timer(100ms, std::bind(&OdometryPublisher::updateOdometry, this));
        
        RCLCPP_INFO(this->get_logger(), "Odometry publisher initialized");
    }

private:
    void setupEncoderPins()
    {
        // エンコーダピンの設定
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
            RCLCPP_ERROR(this->get_logger(), "Left encoder interrupt setup error");
        }
        if (wiringPiISR(RIGHT_ENCODER_PIN_A, INT_EDGE_BOTH, &rightEncoderISR) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Right encoder interrupt setup error");
        }
    }

    void updateOdometry()
    {
        auto current_time = this->get_clock()->now();
        
        // エンコーダの差分を計算
        int delta_left = left_encoder_count_ - prev_left_encoder_;
        int delta_right = right_encoder_count_ - prev_right_encoder_;
        
        // エンコーダ値を更新
        prev_left_encoder_ = left_encoder_count_;
        prev_right_encoder_ = right_encoder_count_;

        // 距離に変換 (メートル)
        double delta_left_dist = 2.0 * M_PI * wheel_radius_ * delta_left / encoder_resolution_;
        double delta_right_dist = 2.0 * M_PI * wheel_radius_ * delta_right / encoder_resolution_;
        
        // 移動距離と回転角の計算
        double delta_dist = (delta_right_dist + delta_left_dist) / 2.0;
        double delta_theta = (delta_right_dist - delta_left_dist) / wheel_base_;

        // 位置と向きの更新
        theta_ += delta_theta;
        x_ += delta_dist * cos(theta_);
        y_ += delta_dist * sin(theta_);

        // デバッグ情報の出力
        auto debug_msg = std_msgs::msg::String();
        debug_msg.data = "Left: " + std::to_string(left_encoder_count_) + 
                        ", Right: " + std::to_string(right_encoder_count_) +
                        ", X: " + std::to_string(x_) +
                        ", Y: " + std::to_string(y_) +
                        ", Theta: " + std::to_string(theta_);
        debug_pub_->publish(debug_msg);

        publishOdometry(current_time, delta_dist, delta_theta);
    }

    void publishOdometry(const rclcpp::Time& current_time, double delta_dist, double delta_theta)
    {
        // Odometryメッセージの作成
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        // 位置の設定
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        // 向きをクォータニオンに変換
        double cy = cos(theta_ * 0.5);
        double sy = sin(theta_ * 0.5);
        odom.pose.pose.orientation.x = 0.0;
        odom.pose.pose.orientation.y = 0.0;
        odom.pose.pose.orientation.z = sy;
        odom.pose.pose.orientation.w = cy;

        // 速度の計算と設定
        double dt = 0.1;  // タイマーの周期 (100ms)
        odom.twist.twist.linear.x = delta_dist / dt;
        odom.twist.twist.angular.z = delta_theta / dt;

        // 共分散行列の設定
        for (size_t i = 0; i < 36; i++) {
            odom.pose.covariance[i] = 0.0;
            odom.twist.covariance[i] = 0.0;
        }
        // 位置の不確実性
        odom.pose.covariance[0] = 0.01;  // x
        odom.pose.covariance[7] = 0.01;  // y
        odom.pose.covariance[35] = 0.01; // theta
        // 速度の不確実性
        odom.twist.covariance[0] = 0.01;  // linear x
        odom.twist.covariance[35] = 0.01; // angular z

        // パブリッシュ
        odom_pub_->publish(odom);

        // TFブロードキャスト
        geometry_msgs::msg::TransformStamped transform;
        transform.header = odom.header;
        transform.child_frame_id = odom.child_frame_id;
        transform.transform.translation.x = x_;
        transform.transform.translation.y = y_;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation = odom.pose.pose.orientation;
        
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

    // エンコーダ関連のピン設定
    static const int LEFT_ENCODER_PIN_A = 17;
    static const int LEFT_ENCODER_PIN_B = 18;
    static const int RIGHT_ENCODER_PIN_A = 22;
    static const int RIGHT_ENCODER_PIN_B = 23;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    static volatile int left_encoder_count_;
    static volatile int right_encoder_count_;
    int prev_left_encoder_;
    int prev_right_encoder_;
    
    // ロボットの位置と向き
    double x_;
    double y_;
    double theta_;

    // パラメータ
    double wheel_radius_;
    double wheel_base_;
    int encoder_resolution_;
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