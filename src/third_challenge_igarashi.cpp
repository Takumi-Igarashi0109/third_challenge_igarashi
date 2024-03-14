#include "third_challenge_igarashi/third_challenge_igarashi.hpp"

using namespace std::chrono_literals;

/*
    ・ros2_yoloから検出結果を受け取る
        ・topic: /ros2_yolo/box (Float32MultiArray)
            idx0: stamp.sec
            idx1: stamp.nanosec
            idx2~5: xmin, ymin, xmax, ymax (pixel 単位)
            idx6: reliability
            idx7: class
    ・検出結果を世界座標に変換する際は/color/camera_infoの情報を利用
        ・See: https://mem-archive.com/2018/10/13/post-682/
    ・検出結果をもとに人の方にルンバを旋回させる
        ・人の方位をルンバの旋回に依存しない座標系に変換し、保持
        ・タイマーを使って目標方位に旋回させる
*/
thirdChallengeIgarashi::thirdChallengeIgarashi() : Node("third_challenge_igarashi")
{
    omega_ = this->declare_parameter<double>("omega",0.5);

    timer_ = this->create_wall_timer (0.5s, 
    std::bind(&thirdChallengeIgarashi::timer_callback, this));    

    box_sub_= this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/ros2_yolo/box", rclcpp::QoS(1).reliable(),
    std::bind(&thirdChallengeIgarashi::box_callback, this, std::placeholders::_1));

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/color/camera_info", rclcpp::QoS(1).reliable(),
    std::bind(&thirdChallengeIgarashi::camera_info_callback, this, std::placeholders::_1));

    
    cmd_vel_pub_ = this->create_publisher<roomba_500driver_meiji::msg::RoombaCtrl>(
    "/roomba/control", rclcpp::QoS(1).reliable());


}

// 検出結果のバウンディングボックスから，odom座標系で表される目標角度を計算する
void thirdChallengeIgarashi::box_callback(const std_msgs::msg::Float32MultiArray& msg)
{
    if (msg.data.size()>2){
        x_callback = true;
    }else{
        x_callback = false;
    }
    x=(msg.data[2]+msg.data[4])/2;
}

// カメラパラメータを取得し，カメラモデルを生成する
void thirdChallengeIgarashi::camera_info_callback(const sensor_msgs::msg::CameraInfo& msg)
{   
    camera_info_.fromCameraInfo(msg);
}

// 一定の周期で呼び出され，制御指令を生成する関数
// ルンバを目標方位角の方に旋回させる
void thirdChallengeIgarashi::timer_callback()
{
    set_cmd_vel();
}

void thirdChallengeIgarashi::run(float velocity, float omega)
{
    // roombaの制御モード
    // 基本的に11（DRIVE_DIRECT）固定で良い
    cmd_vel_.mode = 11;

    // 並進速度を指定
    cmd_vel_.cntl.linear.x  = velocity;
    // 旋回速度を指定
    cmd_vel_.cntl.angular.z = omega;

    // cmd_velをpublish
    // <publisher名>->publish(<変数名>);
    cmd_vel_pub_ -> publish(cmd_vel_);
}

void thirdChallengeIgarashi::set_cmd_vel()
{
    // 計算した制御入力はrun()に渡すこと
    if(is_goal() && x_callback)
    {
        if(thita_ < 0.0){
            run(0.0, omega_);
            printf("1");
        } else {
            run(0.0, -omega_);
            printf("2");
        }
    } else {
        run(0.0, 0.0);
        printf("3");
    }
    printf("  %f\n",thita_);
}

bool thirdChallengeIgarashi::is_goal()
{
    
    thita_ = atan((x-camera_info_.cx())/(1*camera_info_.fx()));

    if( -0.5 < thita_ && thita_ < 0.5 ){
        return false;
    } else {
        return true;
    }
}