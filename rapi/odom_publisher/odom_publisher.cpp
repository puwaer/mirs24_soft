#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <wiringPi.h>
#include <chrono>

class OdometryPublisher : public rclcpp::Node
{
public:
    OdometryPublisher()
        : Node("odometry_publisher"), 
          x_(0.0), y_(0.0), theta_(0.0)
    {
        // GPIOの設定
        setup_gpio();

        // パブリッシャーの作成
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // タイマーで定期的にオドメトリをパブリッシュ
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&OdometryPublisher::publish_odometry, this));
        
        // TFブロードキャスター
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    static void left_encoder_isr()
    {
        // インスタンスを取得
        OdometryPublisher* instance = get_instance();
        
        if (digitalRead(instance->left_encoder_b_pin_)) {
            instance->left_encoder_++;
        } else {
            instance->left_encoder_--;
        }
    }

    static void right_encoder_isr()
    {
        // インスタンスを取得
        OdometryPublisher* instance = get_instance();

        if (digitalRead(instance->right_encoder_b_pin_)) {
            instance->right_encoder_++;
        } else {
            instance->right_encoder_--;
        }
    }

    static OdometryPublisher* get_instance()
    {
        static OdometryPublisher instance;
        return &instance;
    }

private:
    void setup_gpio()
    {
        // GPIOピンの初期化
        wiringPiSetup();

        // 左エンコーダーのA相とB相
        pinMode(left_encoder_a_pin_, INPUT);
        pinMode(left_encoder_b_pin_, INPUT);
        
        // 右エンコーダーのA相とB相
        pinMode(right_encoder_a_pin_, INPUT);
        pinMode(right_encoder_b_pin_, INPUT);
        
        // プルアップ抵抗の設定
        pullUpDnControl(left_encoder_a_pin_, PUD_UP);
        pullUpDnControl(left_encoder_b_pin_, PUD_UP);
        pullUpDnControl(right_encoder_a_pin_, PUD_UP);
        pullUpDnControl(right_encoder_b_pin_, PUD_UP);

        // エンコーダーの割り込み設定
        wiringPiISR(left_encoder_a_pin_, INT_EDGE_BOTH, &OdometryPublisher::left_encoder_isr);
        wiringPiISR(right_encoder_a_pin_, INT_EDGE_BOTH, &OdometryPublisher::right_encoder_isr);
    }

    void publish_odometry()
    {
        // オドメトリの計算およびパブリッシュ処理
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 車両の状態
    double x_, y_, theta_;
    
    // GPIOピン
    const int left_encoder_a_pin_ = 0;  // 左エンコーダーA相
    const int left_encoder_b_pin_ = 1;  // 左エンコーダーB相
    const int right_encoder_a_pin_ = 2; // 右エンコーダーA相
    const int right_encoder_b_pin_ = 3; // 右エンコーダーB相

    // エンコーダーの状態
    static int last_left_encoder_;
    static int last_right_encoder_;
    static int left_encoder_;
    static int right_encoder_;
};

// 静的メンバ変数の初期化
int OdometryPublisher::last_left_encoder_ = 0;
int OdometryPublisher::last_right_encoder_ = 0;
int OdometryPublisher::left_encoder_ = 0;
int OdometryPublisher::right_encoder_ = 0;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryPublisher>());
    rclcpp::shutdown();
    return 0;
}
