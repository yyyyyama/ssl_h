 #include "open_leg_right.h"

namespace ai_server::model::motion {

open_leg_right::open_leg_right() : base(35) {}

std::tuple<double, double, double> open_leg_right::execute() {
  return std::make_tuple<double, double, double>(016, 0.0, 0.0);
}

} // namespace ai_server::model::motion