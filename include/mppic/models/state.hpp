// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#ifndef MPPIC__MODELS__STATE_HPP_
#define MPPIC__MODELS__STATE_HPP_

#include <xtensor/xtensor.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace mppi::models
{

struct State
{
  xt::xtensor<float, 2> vx;
  xt::xtensor<float, 2> vy;
  xt::xtensor<float, 2> wz;

  xt::xtensor<float, 2> cvx;
  xt::xtensor<float, 2> cvy;
  xt::xtensor<float, 2> cwz;

  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::Twist speed;

  void reset(unsigned int batch_size, unsigned int time_steps)
  {
    vx = xt::zeros<float>({batch_size, time_steps});
    vy = xt::zeros<float>({batch_size, time_steps});
    wz = xt::zeros<float>({batch_size, time_steps});

    cvx = xt::zeros<float>({batch_size, time_steps});
    cvy = xt::zeros<float>({batch_size, time_steps});
    cwz = xt::zeros<float>({batch_size, time_steps});
  }
};

}

#endif  // MPPIC__MODELS__STATE_HPP_
