#pragma once

namespace mppi::context {

#ifdef PROFILE
constexpr inline bool profile = true;
#else
constexpr inline bool profile = false;
#endif

}
