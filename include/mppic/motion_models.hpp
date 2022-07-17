// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research

#ifndef MPPIC__MOTION_MODELS_HPP_
#define MPPIC__MOTION_MODELS_HPP_

#include <cstdint>

#include "mppic/models/state.hpp"
#include <xtensor/xmath.hpp>
#include <xtensor/xmasked_view.hpp>

#include "mppic/tools/parameters_handler.hpp"

namespace mppi
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
  virtual xt::xtensor<float, 2> predict(
    const xt::xtensor<float, 2> & /* state */, const models::StateIdxes & /* idx */) { 
      throw std::runtime_error("Predict not implemened");
  };


  virtual bool isHolonomic() = 0;
  virtual bool isNaive() {return true;}
  virtual void applyConstraints(models::State & /*state*/) {}
};

class AckermannMotionModel : public MotionModel
{
public:
  explicit AckermannMotionModel(ParametersHandler * param_handler)
  {
    auto getParam = param_handler->getParamGetter("AckermannConstraints");
    getParam(min_turning_r_, "min_turning_r", 0.2);
  }

  bool isHolonomic() override
  {
    return false;
  }

  void applyConstraints(models::State & state) override
  {
    auto v = state.getVelocitiesVX();
    auto w = state.getVelocitiesWZ();

    auto view = xt::masked_view(w, v / w > min_turning_r_);
    view = xt::sign(v) / min_turning_r_;
  }

private:
  double min_turning_r_{0};
};

class DiffDriveMotionModel : public MotionModel
{
public:
  DiffDriveMotionModel() = default;

  bool isHolonomic() override
  {
    return false;
  }
};

class OmniMotionModel : public MotionModel
{
public:
  OmniMotionModel() = default;

  bool isHolonomic() override
  {
    return true;
  }

};

}  // namespace mppi::models

#endif  // MPPIC__MOTION_MODELS_HPP_
