// Copyright 2022 FastSense, Samsung Research
#pragma once

#include "mppic/models/state.hpp"
#include <xtensor/xmath.hpp>
#include <xtensor/xmasked_view.hpp>

#include "mppic/parameters_handler.hpp"

namespace mppi
{

enum class MotionModelType
{
  DiffDrive,
  Omni,
  Ackermann
};


inline bool isHolonomic(MotionModelType type)
{
  switch (type) {
    case MotionModelType::Omni:
      return true;
    case MotionModelType::Ackermann:
    case MotionModelType::DiffDrive:
      return false;
  }

  return false;
}

class IMotionModel
{
public:
  virtual ~IMotionModel() = default;

  virtual MotionModelType getMotionModelType() = 0;

  virtual void applyConstraints(models::State & /*state*/) {}
  bool isHolonomic() {return ::mppi::isHolonomic(getMotionModelType());}
};

class AckermannMotionModel final : public IMotionModel
{
public:
  explicit AckermannMotionModel(ParametersHandler * param_handler)
  {

    auto getParam = param_handler->getParamGetter("AckermannConstraints");
    getParam(min_turning_r_, "min_turning_r", 0.2);
  }

  MotionModelType getMotionModelType() override
  {
    return MotionModelType::Ackermann;
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

class OmniMotionModel final : public IMotionModel
{
public:
  OmniMotionModel() = default;

  MotionModelType getMotionModelType() override
  {
    return MotionModelType::Omni;
  }

};

class DiffDriveMotionModel final : public IMotionModel
{
public:
  DiffDriveMotionModel() = default;

  MotionModelType getMotionModelType() override
  {
    return MotionModelType::DiffDrive;
  }

};

}  // namespace mppi::models
