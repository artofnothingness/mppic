#pragma once

#include <string_view>
#include <unordered_map>

namespace mppi::optimization {
enum class MotionModel : uint8_t { Omni, DiffDrive, Carlike };

inline bool isHolonomic(MotionModel motion_model)
{
  return (motion_model == MotionModel::Omni) ? true : false;
}

const inline std::unordered_map<std::string, MotionModel> motion_model_name_map_ = {
  { "diff", MotionModel::DiffDrive },
  { "carlike", MotionModel::Carlike },
  { "omni", MotionModel::Omni }
};

const inline std::unordered_map<MotionModel, unsigned int> motion_model_control_dim_map_ = {
  { MotionModel::DiffDrive, 2U },
  { MotionModel::Carlike, 2U },
  { MotionModel::Omni, 3U }
};

}// namespace mppi::optimization
