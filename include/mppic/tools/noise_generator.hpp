// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research

#ifndef MPPIC__NOISE_GENERATOR_HPP_
#define MPPIC__NOISE_GENERATOR_HPP_

#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

#include "mppic/models/optimizer_settings.hpp"
#include "mppic/models/state.hpp"

namespace mppi
{

class NoiseGenerator
{
public:
  NoiseGenerator() = default;

  /**
   * @brief Initialize noise generator with settings and model types
   * @param settings Settings of controller
   * @param is_holonomic If base is holonomic
   */
  void initialize(mppi::models::OptimizerSettings & settings, bool is_holonomic);

  /**
   * @brief Shutdown noise generator thread
   */
  void shutdown();

  /**
   * @brief Signal to the noise thread the controller is ready to generate a new
   * noised control for the next iteration
   */
  void generateNextNoises();

  /**
   * @brief Get the current noises
   * @return noises Reference to current noises
   */
  xt::xtensor<double, 3> & getNoises();

  /**
   * @brief Reset noise generator with settings and model types
   * @param settings Settings of controller
   * @param is_holonomic If base is holonomic
   */
  void reset(mppi::models::OptimizerSettings & settings, bool is_holonomic);

protected:
  /**
   * @brief Thread to execute noise generation process
   */
  void noiseThread();

  /**
   * @brief Generate random controls by gaussian noise with mean in
   * control_sequence_
   *
   * @return tensor of shape [ batch_size_, time_steps_, 2]
   * where 2 stands for v, w
   */
  void generateNoisedControls();

  xt::xtensor<double, 3> noises_;

  mppi::models::OptimizerSettings settings_;
  bool is_holonomic_;

  std::thread noise_thread_;
  std::condition_variable noise_cond_;
  std::mutex noise_lock_;
  bool active_{false}, ready_{false};
};

}  // namespace mppi

#endif  // MPPIC__NOISE_GENERATOR_HPP_
