// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#ifndef MPPIC__MOTION_MODELS_HPP_
#define MPPIC__MOTION_MODELS_HPP_

#include <cstdint>

#include "mppic/optimizers/xtensor/models/state.hpp"

namespace mppi::xtensor
{

class MotionModel
{
public:
  MotionModel() = default;
  virtual ~MotionModel() = default;

  /**
   * @brief Predict velocities for given trajectories the next time step
   *
   * @param state for given time_step, tensor of shape
   * [batch_size, ...] where last dim could be 5 or 7 depending on motion model used
   *
   * @return predicted velocities of the robot: tensor of shape [batch_size, ... ]
   * where last dim could be 2 or 3 depending on motion model used
   */
  virtual xt::xtensor<double, 2> predict(
    const xt::xtensor<double, 2> & state, const models::StateIdxes & idx)
  {
    return xt::view(state, xt::all(), xt::range(idx.cbegin(), idx.cend()));
  }

  virtual bool isHolonomic() const = 0;
};

class DiffDriveMotionModel : public MotionModel
{
public:
  bool isHolonomic() const override {return false;}
};

class OmniMotionModel : public MotionModel
{
public:
  bool isHolonomic() const override {return true;}
};

class AckermannMotionModel : public MotionModel
{
public:
  xt::xtensor<double, 2> predict(
    const xt::xtensor<double, 2> & /*state*/, const models::StateIdxes & /*idx*/) override
  {
    throw std::runtime_error("Ackermann motion model not yet implemented");
  }

  bool isHolonomic() const override {return false;}
};

}  // namespace mppi::xtensor

#endif  // MPPIC__MOTION_MODELS_HPP_
