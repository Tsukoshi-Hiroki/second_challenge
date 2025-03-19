#ifndef SECOND_CHALLENGE_HPP
#define SECOND_CHALLENGE_HPP

#include <chrono>

using namespace std::chrono_literals;

class SecondChallenge : public rclcpp::Node
{
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // SECOND_CHALLENGE_HPP
