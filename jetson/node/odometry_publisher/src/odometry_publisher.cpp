#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <chrono>

class OdometryPublisher : public rclcpp::Node
{
public:
    OdometryPublisher()
        : Node("odometry_publisher"), last_left_encoder_(0), last_right_encoder_(0),
          x_(0.0), y_(0.0), theta_(0.0)
    {
        // サブスクライバーの作成
        left_encoder_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/L_encoder", 10, std::bind(&OdometryPublisher::left_encoder_callback, this, std::placeholders::_1));
        right_encoder_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/R_encoder", 10, std::bind(&OdometryPublisher::right_encoder_callback, this, std::placeholders::_1));

        // パブリッシャーの作成
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // タイマーで定期的にオドメトリをパブリッシュ
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&OdometryPublisher::publish_odometry, this));

        // TFブロードキャスター
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 車輪の仕様
        wheel_radius_ = 0.05;  // 車輪半径 (m)
        wheel_base_ = 0.3;     // 車輪間距離 (m)
        ticks_per_revolution_ = 4096; // エンコーダの1回転あたりのティック数
    }

private:
    void left_encoder_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        left_encoder_ = msg->data;
    }

    void right_encoder_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        right_encoder_ = msg->data;
    }

    void publish_odometry()
    {
        // エンコーダの変化量を計算
        int delta_left = left_encoder_ - last_left_encoder_;
        int delta_right = right_encoder_ - last_right_encoder_;

        last_left_encoder_ = left_encoder_;
        last_right_encoder_ = right_encoder_;

        // 車輪の回転角度（ラジアン）を計算
        double delta_left_rad = (delta_left / static_cast<double>(ticks_per_revolution_)) * 2.0 * M_PI;
        double delta_right_rad = (delta_right / static_cast<double>(ticks_per_revolution_)) * 2.0 * M_PI;

        // それぞれの車輪の移動距離を計算
        double left_distance = delta_left_rad * wheel_radius_;
        double right_distance = delta_right_rad * wheel_radius_;

        // ロボットの移動距離と回転量を計算
        double delta_distance = (left_distance + right_distance) / 2.0;
        double delta_theta = (right_distance - left_distance) / wheel_base_;

        // ロボットの位置と姿勢を更新
        x_ += delta_distance * cos(theta_);
        y_ += delta_distance * sin(theta_);
        theta_ += delta_theta;

        // オドメトリメッセージの作成
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = "odom";

        // 位置を設定
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        //odom_msg.pose.pose.position.z = 0.0;

        // オリエンテーション（四元数）
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.child_frame_id = "base_link";

        // 速度を設定（今回は仮で0としていますが、実際はエンコーダの変化量から計算します）
        odom_msg.twist.twist.linear.x = delta_distance / 0.1;  // 0.1秒周期なので0.1で割る
        odom_msg.twist.twist.angular.z = delta_theta / 0.1;

        // オドメトリをパブリッシュ
        odom_pub_->publish(odom_msg);

        // TF変換のパブリッシュ
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = this->get_clock()->now();
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_link";

        odom_tf.transform.translation.x = x_;
        odom_tf.transform.translation.y = y_;
        //odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation.x = q.x();
        odom_tf.transform.rotation.y = q.y();
        odom_tf.transform.rotation.z = q.z();
        odom_tf.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(odom_tf);
    }

    // サブスクライバーとパブリッシャー
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_encoder_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_encoder_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // タイマーとTFブロードキャスター
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // エンコーダの変数
    int left_encoder_, right_encoder_;
    int last_left_encoder_, last_right_encoder_;

    // ロボットの位置と姿勢
    double x_, y_, theta_;

    // ロボットのパラメータ
    double wheel_radius_, wheel_base_;
    int ticks_per_revolution_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryPublisher>());
    rclcpp::shutdown();
    return 0;
}
