#include "open_leg_left.h"

namespace ai_server::model::motion {

open_leg_left::open_leg_left() : base(34) {}
//open_leg_left::open_leg_left() : base(16) {}

std::tuple<double, double, double> open_leg_left::execute() {
  return std::make_tuple<double, double, double>(0.0, 0.0, 0.0);
}

} // namespace ai_server::model::motion