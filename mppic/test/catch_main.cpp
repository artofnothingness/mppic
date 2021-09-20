#define CATCH_CONFIG_MAIN // This tells the catch header to generate a main

#include <catch2/catch.hpp>

#include <rclcpp/rclcpp.hpp>

class RosLockGuard 
{
public:
  RosLockGuard() { rclcpp::init(0, nullptr); }
  ~RosLockGuard() { rclcpp::shutdown(); }
};
RosLockGuard g_rclcpp;
