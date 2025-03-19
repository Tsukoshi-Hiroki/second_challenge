#include "second_challenge/second_challenge.hpp"

SecondChallenge::SecondChallenge()
: Node("second_challenge")
{
  // timer
  timer_ = this->create_wall_timer(<周期間隔>, std::bind(&<class名>::<callback関数名, this));
}

void SecondChallenge::timer_callback()
{
    // 一定周期で行う処理を書く
}

