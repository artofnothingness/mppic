// Copyright 2022 FastSense, Samsung Research
#pragma once

#include "mppic/models/state.hpp"
#include <xtensor/xmath.hpp>
#include <xtensor/xmasked_view.hpp>

#include "mppic/parameters_handler.hpp"

namespace mppi
{

class IMotionModel
{
public:
  virtual ~IMotionModel() = default;

  virtual bool isHolonomic() = 0;
  virtual void applyConstraints(models::State & /*state*/) {}
};

class AckermannMotionModel final : public IMotionModel
{
public:
  explicit AckermannMotionModel(ParametersHandler * param_handler)
  {
    auto getParam = param_handler->getParamGetter("AckermannConstraints");
    getParam(min_turning_r_, "min_turning_r", 0.2);
  }

  bool isHolonomic() override {
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

class DiffDriveMotionModel final : public IMotionModel
{
public:
  DiffDriveMotionModel() = default;

  bool isHolonomic() override {
      return false;
  }
};

class OmniMotionModel final : public IMotionModel
{
public:
  OmniMotionModel() = default;

  bool isHolonomic() override {
      return true;
  }

};

}  // namespace mppi::models
