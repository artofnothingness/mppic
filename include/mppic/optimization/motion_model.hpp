#ifndef MPPIC__OPTIMIZATION__MOTION_MODELS_HPP_
#define MPPIC__OPTIMIZATION__MOTION_MODELS_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>

namespace mppi
{

enum class MotionModel : uint8_t { Omni, DiffDrive, Carlike };

inline bool isHolonomic(const MotionModel & motion_model)
{
  return (motion_model == MotionModel::Omni) ? true : false;
}

const inline std::unordered_map<std::string, MotionModel> MOTION_MODEL_NAMES_MAP = {
  {"diff", MotionModel::DiffDrive}, {"carlike", MotionModel::Carlike}, {"omni", MotionModel::Omni}};

} // namespace mppi

#endif  // MPPIC__OPTIMIZATION__MOTION_MODELS_HPP_
