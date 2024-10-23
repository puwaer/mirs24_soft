#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <wiringPi.h>

// エンコーダピン設定
#define LEFT_ENCODER_PIN_A 17
#define LEFT_ENCODER_PIN_B 18
#define RIGHT_ENCODER_PIN_A 22
#define RIGHT_ENCODER_PIN_B 23

// ロボットパラメータ
#define WHEEL_RADIUS 0.05
#define WHEEL_BASE 0.25
#define ENCODER_RESOLUTION 360

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
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        if (wiringPiSetupGpio() == -1) {
            RCLCPP_ERROR(this->get_logger(), "GPIO initialization failed");
            return;
        }

        // GPIOピン設定
        for (int pin : {LEFT_ENCODER_PIN_A, LEFT_ENCODER_PIN_B, RIGHT_ENCODER_PIN_A, RIGHT_ENCODER_PIN_B}) {
            pinMode(pin, INPUT);
            pullUpDnControl(pin, PUD_UP);
        }

        // 割り込み設定
        wiringPiISR(LEFT_ENCODER_PIN_A, INT_EDGE_BOTH, &leftEncoderISR);
        wiringPiISR(RIGHT_ENCODER_PIN_A, INT_EDGE_BOTH, &rightEncoderISR);

        timer_ = this->create_wall_timer(100ms, std::bind(&OdometryPublisher::updateOdometry, this));
    }

private:
    void updateOdometry()
    {
        auto current_time = this->get_clock()->now();
        
        // エンコーダ差分計算
        int delta_left = left_encoder_count_ - prev_left_encoder_;
        int delta_right = right_encoder_count_ - prev_right_encoder_;
        prev_left_encoder_ = left_encoder_count_;
        prev_right_encoder_ = right_encoder_count_;

        // 移動距離計算
        double delta_left_rad = (delta_left / static_cast<double>(ENCODER_RESOLUTION)) * 2.0 * M_PI;
        double delta_right_rad = (delta_right / static_cast<double>(ENCODER_RESOLUTION)) * 2.0 * M_PI;
        double left_distance = delta_left_rad * WHEEL_RADIUS;
        double right_distance = delta_right_rad * WHEEL_RADIUS;

        // オドメトリ計算
        double delta_distance = (left_distance + right_distance) / 2.0;
        double delta_theta = (right_distance - left_distance) / WHEEL_BASE;
        x_ += delta_distance * cos(theta_);
        y_ += delta_distance * sin(theta_);
        theta_ += delta_theta;

        // Odometryメッセージ作成
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        // 位置設定
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        // 姿勢設定
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        // 速度設定
        odom.twist.twist.linear.x = delta_distance / 0.1;
        odom.twist.twist.angular.z = delta_theta / 0.1;

        // Odometryパブリッシュ
        odom_pub_->publish(odom);

        // TF変換パブリッシュ
        geometry_msgs::msg::TransformStamped transform;
        transform.header = odom.header;
        transform.child_frame_id = odom.child_frame_id;
        transform.transform.translation.x = x_;
        transform.transform.translation.y = y_;
        transform.transform.translation.z = 0.0;
        
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(transform);
    }

    static void leftEncoderISR()
    {
        static int last_state = 0;
        int state = digitalRead(LEFT_ENCODER_PIN_A);
        int b_state = digitalRead(LEFT_ENCODER_PIN_B);
        
        if (state != last_state) {
            left_encoder_count_ += (b_state != state) ? 1 : -1;
        }
        last_state = state;
    }

    static void rightEncoderISR()
    {
        static int last_state = 0;
        int state = digitalRead(RIGHT_ENCODER_PIN_A);
        int b_state = digitalRead(RIGHT_ENCODER_PIN_B);
        
        if (state != last_state) {
            right_encoder_count_ += (b_state != state) ? 1 : -1;
        }
        last_state = state;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    static volatile int left_encoder_count_;
    static volatile int right_encoder_count_;
    int prev_left_encoder_;
    int prev_right_encoder_;
    double x_, y_, theta_;
};

volatile int OdometryPublisher::left_encoder_count_ = 0;
volatile int OdometryPublisher::right_encoder_count_ = 0;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryPublisher>());
    rclcpp::shutdown();
    return 0;
}