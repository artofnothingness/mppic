#define CATCH_CONFIG_MAIN // This tells the catch header to generate a main

#ifdef DO_BENCHMARKS
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#endif

#include <catch2/catch.hpp>
#include <rclcpp/rclcpp.hpp>

class RosLockGuard 
{
public:
  RosLockGuard() { rclcpp::init(0, nullptr); }
  ~RosLockGuard() { rclcpp::shutdown(); }
};
RosLockGuard g_rclcpp;
