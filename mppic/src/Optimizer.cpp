#include "mppi/Optimizer.hpp"

namespace ultra::mppi {

auto Optimizer::operator()(PoseStamped const &pose, Twist const &twist,
                               Path const &path) -> TwistStamped {

  (void)pose;
  (void)twist;
  (void)path;

  return TwistStamped{};
}
} // namespace ultra
