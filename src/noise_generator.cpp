// Copyright 2022 @artofnothingness Alexey Budyakov, Samsung Research
#include "mppic/tools/noise_generator.hpp"

#include <memory>
#include <mutex>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>

namespace mppi
{

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

std::tuple<xt::xtensor<float, 2> &, xt::xtensor<float, 2> &, xt::xtensor<float, 2> &>
NoiseGenerator::getNoises()
{
  std::unique_lock<std::mutex> guard(noise_lock_);
  return std::tie(noises_vx_, noises_vy_, noises_wz_);
}

void NoiseGenerator::reset(mppi::models::OptimizerSettings & settings, bool is_holonomic)
{
  settings_ = settings;
  is_holonomic_ = is_holonomic;

  // Recompute the noises on reset, initialization, and fallback
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    noises_vx_ = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    noises_vy_ = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    noises_wz_ = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
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

  noises_vx_ = xt::random::randn<float>({s.batch_size, s.time_steps}, 0.0, s.sampling_std.vx);
  noises_wz_ = xt::random::randn<float>({s.batch_size, s.time_steps}, 0.0, s.sampling_std.wz);
  if (is_holonomic_) {
    noises_vy_ = xt::random::randn<float>({s.batch_size, s.time_steps}, 0.0, s.sampling_std.vy);
  }
}

}  // namespace mppi
