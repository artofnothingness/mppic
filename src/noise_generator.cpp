// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#include "mppic/tools/noise_generator.hpp"

#include <memory>
#include <mutex>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xnoalias.hpp>

namespace mppi
{

using xt::placeholders::_;

void NoiseGenerator::initialize(mppi::models::OptimizerSettings & settings, bool is_holonomic)
{
  settings_ = settings;
  is_holonomic_ = is_holonomic;
  active_ = true;
  noise_thread_ = std::thread(std::bind(&NoiseGenerator::noiseThread, this));
}

void NoiseGenerator::shutdown()
{
  active_ = false;
  ready_ = true;
  noise_cond_.notify_all();
  noise_thread_.join();
}

void NoiseGenerator::generateNextNoises()
{
  // Trigger the thread to run in parallel to this iteration
  // to generate the next iteration's noises.
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    ready_ = true;
  }
  noise_cond_.notify_all();
}

void NoiseGenerator::setNoisedControls(
  models::State & state,
  const models::ControlSequence & control_sequence)
{
  std::unique_lock<std::mutex> guard(noise_lock_);

  // Where to divide set of noises between zero- and U-meaned controls
  const int division = ceil((1.0 - settings_.zero_mean_percentage) * settings_.batch_size);

  auto applyNoises = [&](auto & axis_noises, auto & axis_state, auto & axis_control) -> void {
      auto noises = xt::view(axis_noises, xt::range(0, division), xt::all());
      auto zero_mean_noises = xt::view(axis_noises, xt::range(division, _), xt::all());
      auto noised_controls = xt::view(axis_state, xt::range(0, division), xt::all());
      auto zero_mean_controls = xt::view(axis_state, xt::range(division, _), xt::all());
      xt::noalias(noised_controls) = axis_control + noises;
      xt::noalias(zero_mean_controls) = zero_mean_noises;
    };

  applyNoises(noises_vx_, state.cvx, control_sequence.vx);
  applyNoises(noises_vy_, state.cvy, control_sequence.vy);
  applyNoises(noises_wz_, state.cwz, control_sequence.wz);
}

void NoiseGenerator::reset(mppi::models::OptimizerSettings & settings, bool is_holonomic)
{
  settings_ = settings;
  is_holonomic_ = is_holonomic;

  // Recompute the noises on reset, initialization, and fallback
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    xt::noalias(noises_vx_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    xt::noalias(noises_vy_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    xt::noalias(noises_wz_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    ready_ = true;
  }
  noise_cond_.notify_all();
}

void NoiseGenerator::noiseThread()
{
  do {
    std::unique_lock<std::mutex> guard(noise_lock_);
    noise_cond_.wait(guard, [this]() {return ready_;});
    ready_ = false;
    generateNoisedControls();
  } while (active_);
}

void NoiseGenerator::generateNoisedControls()
{
  auto & s = settings_;

  xt::noalias(noises_vx_) = xt::random::randn<float>(
    {s.batch_size, s.time_steps}, 0.0,
    s.sampling_std.vx);
  xt::noalias(noises_wz_) = xt::random::randn<float>(
    {s.batch_size, s.time_steps}, 0.0,
    s.sampling_std.wz);
  if (is_holonomic_) {
    xt::noalias(noises_vy_) = xt::random::randn<float>(
      {s.batch_size, s.time_steps}, 0.0,
      s.sampling_std.vy);
  }
}

}  // namespace mppi
