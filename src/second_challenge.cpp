#include "second_challenge/second_challenge.hpp"

SecondChallenge::SecondChallenge() : Node("second_challenge")
{
  // timer
  timer_ = this->create_wall_timer(100ms, std::bind(&SecondChallenge::timer_callback, this));

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::QoS(1).reliable(),
      std::bind(&SecondChallenge::scan_callback, this, std::placeholders::_1));

  cmd_vel_pub_ =
      this->create_publisher<roomba_500driver_meiji::msg::RoombaCtrl>(
          "/roomba/control", rclcpp::QoS(1).reliable());
}


  // 一定周期で行う処理を書く
void SecondChallenge::timer_callback()
{
  if(scan_)
  {
    if(calc_min_dist() > 1.0)
    {
      run(0.1,0.0);
    }else{
      run(0.0,0.0);
    }

  }else
  {
    run(0.0,0.0);
  }
}

// Lidarのコールバック
void SecondChallenge::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  scan_ = *msg;
}

//センサの最小値を算出
double SecondChallenge::calc_min_dist()
{
  double all_angle = scan_.value().angle_max - scan_.value().angle_min;
  double min_range ;

  RCLCPP_INFO(this->get_logger(), "angle_min = %lf",scan_.value().angle_min);
  RCLCPP_INFO(this->get_logger(), "angle_max = %lf",scan_.value().angle_max);
  RCLCPP_INFO(this->get_logger(), "all_angle = %lf",all_angle);
  RCLCPP_INFO(this->get_logger(), "angle_increment = %lf",scan_.value().angle_increment);
  RCLCPP_INFO(this->get_logger(), "range_number = %ld",size(scan_.value().ranges));

  int j=0;
  for(int i=421; i<=541; i++)
  {
    range_array[j] = scan_.value().ranges[i];
    j++;
  }

  min_range = *min_element(begin(range_array), end(range_array));
  RCLCPP_INFO(this->get_logger(), "min_range = %lf",min_range);

  return min_range;
}

void SecondChallenge::run(double velocity, double omega) {
  RCLCPP_INFO(this->get_logger(), "run");

  cmd_vel_.mode = 11;
  cmd_vel_.cntl.linear.x = velocity;
  cmd_vel_.cntl.angular.z = omega;

  cmd_vel_pub_->publish(cmd_vel_);
}