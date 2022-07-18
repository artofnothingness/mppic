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

xt::xtensor<double, 3> & NoiseGenerator::getNoises()
{
  std::unique_lock<std::mutex> guard(noise_lock_);
  return noises_;
}

void NoiseGenerator::reset(mppi::models::OptimizerSettings & settings, bool is_holonomic)
{
  settings_ = settings;
  is_holonomic_ = is_holonomic;

  // Recompute the noises on reset, initialization, and fallback
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    noises_ =
      xt::zeros<double>({settings_.batch_size, settings_.time_steps, is_holonomic_ ? 3u : 2u});
    ready_ = true;
  }
  noise_cond_.notify_all();
}

void NoiseGenerator::noiseThread()
{
  do {
    std::unique_lock<std::mutex> guard(noise_lock_);
    noise_cond_.wait(guard, [this](){return ready_;});
    ready_ = false;
    generateNoisedControls();
  } while (active_);
}

void NoiseGenerator::generateNoisedControls()
{
  auto & s = settings_;
  auto vx = xt::view(noises_, xt::all(), xt::all(), 0);
  auto wz = xt::view(noises_, xt::all(), xt::all(), is_holonomic_ ? 2 : 1);

  vx = xt::random::randn<double>({s.batch_size, s.time_steps}, 0.0, s.sampling_std.vx);
  wz = xt::random::randn<double>({s.batch_size, s.time_steps}, 0.0, s.sampling_std.wz);
  if (is_holonomic_) {
    auto vy = xt::view(noises_, xt::all(), xt::all(), 1);
    vy = xt::random::randn<double>({s.batch_size, s.time_steps}, 0.0, s.sampling_std.vy);
  }
}

}  // namespace mppi
