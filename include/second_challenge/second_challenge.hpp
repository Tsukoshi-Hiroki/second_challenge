#ifndef SECOND_CHALLENGE_HPP
#define SECOND_CHALLENGE_HPP

#include <chrono>
#include "roomba_500driver_meiji/msg/roomba_ctrl.hpp"
#include <functional>
#include <memory>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class SecondChallenge : public rclcpp::Node
{
public:
  SecondChallenge();

private:
  // 関数
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void timer_callback();
  void is_goal(std::optional<sensor_msgs::msg::LaserScan> scan_);
  double calc_min_dist();
  void run(double velocity, double omega);

  // pub sub
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<roomba_500driver_meiji::msg::RoombaCtrl>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 変数
  std::optional<sensor_msgs::msg::LaserScan> scan_;
  roomba_500driver_meiji::msg::RoombaCtrl cmd_vel_;
  double goal_dist = 1.0;
  double velocity = 0.0;
  double omega = 0.0;
};

#endif  // SECOND_CHALLENGE_HPP
