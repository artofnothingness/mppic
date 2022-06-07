// Copyright 2022 FastSense, Samsung Research
#pragma once

#include "mppic/models/state.hpp"
#include <xtensor/xmath.hpp>
#include <xtensor/xmasked_view.hpp>

#include "mppic/parameters_handler.hpp"

namespace mppi
{

enum class MotionModelType {
  DiffDrive,
  Omni,
  Ackermann
};


inline bool isHolonomic(MotionModelType type) {
    switch (type) {
    case MotionModelType::Omni:
      return true;
    case MotionModelType::Ackermann:
    case MotionModelType::DiffDrive:
      return false;
    }

  return false;
}

class IModelConstraints {
public:
  virtual ~IModelConstraints() = default;
  virtual void applyConstraints(const models::State & state) = 0;
};

class AckermannConstraints final : public IModelConstraints {
public:
  AckermannConstraints(ParametersHandler * param_handler) {

  auto getParam = param_handler->getParamGetter("AckermannConstraints");
  getParam(min_turning_r_, "min_turning_r", 0.2);
}

void applyConstraints(const models::State & state) override {
  auto v = state.getVelocitiesVX();
  auto w = state.getVelocitiesWZ();

  auto mask = v / w > min_turning_r_; 
  auto w_masked = xt::masked_view(w, mask);
  w_masked = xt::sign(v) /  min_turning_r_;
}

private:
  double min_turning_r_{0};
};

inline std::unique_ptr<IModelConstraints> 
getModelConstraintsUniquePtr(ParametersHandler * param_handler, MotionModelType type) {
    switch (type) {
    case MotionModelType::Omni:
    case MotionModelType::DiffDrive:
      return{};
    case MotionModelType::Ackermann:
      return std::make_unique<AckermannConstraints>(param_handler);
    }

    return {};
}

}  // namespace mppi::models
