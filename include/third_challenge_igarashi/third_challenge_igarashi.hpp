#ifndef THIRD_CHALLENGE_HPP
#define THIRD_CHALLENGE_HPP

#include <rclcpp/rclcpp.hpp>
#include <functional>  // bind & placeholders用
#include <memory>      // SharedPtr用
#include <cmath> // 要らないかも
#include <image_geometry/pinhole_camera_model.h>
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "roomba_500driver_meiji/msg/roomba_ctrl.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

class thirdChallengeIgarashi : public rclcpp::Node
{
    public:
        thirdChallengeIgarashi();

        void box_callback(const std_msgs::msg::Float32MultiArray& msg);
        void camera_info_callback(const sensor_msgs::msg::CameraInfo& msg);
        void timer_callback();
        bool is_goal();                         // 終了判定
    //double calc_distance();                 // 進んだ距離を計算
        void run(float velocity, float omega);  // roombaの制御入力を決定
        void set_cmd_vel(); 

        double omega_ =0.0;
        double thita_ =0.0;
        double x = 0.0;
        bool x_callback;

    private:
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr box_sub_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
        rclcpp::Publisher<roomba_500driver_meiji::msg::RoombaCtrl>::SharedPtr cmd_vel_pub_;
        image_geometry::PinholeCameraModel camera_info_;
        roomba_500driver_meiji::msg::RoombaCtrl cmd_vel_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        geometry_msgs::msg::Quaternion odom_quat_;
        bool is_model_set = false;
        bool detected = false;

        int mode = 11;
        double frontal_threshold;
        double base_omega;
};

#endif  // THIRD_CHALLENGE_HPP