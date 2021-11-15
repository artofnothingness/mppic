#pragma once

#include <cstdint>
#include <tuple>

namespace mppi::idxes {

constexpr uint8_t linear_velocities = 0;
constexpr uint8_t angular_velocities = 1;
constexpr uint8_t control_linear_velocities = 2;
constexpr uint8_t control_angular_velocities = 3;
constexpr uint8_t dt = 3;

constexpr inline std::tuple control_range{control_linear_velocities,
                                          control_angular_velocities + 1};
} // namespace mppi::idxes

namespace mppi::dims {

constexpr uint8_t batches = 3;
constexpr uint8_t control_sequence = batches - 1;

} // namespace mppi::dims
