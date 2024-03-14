#include "third_challenge_igarashi/third_challenge_igarashi.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<thirdChallengeIgarashi>());
  rclcpp::shutdown();
  return 0;
}