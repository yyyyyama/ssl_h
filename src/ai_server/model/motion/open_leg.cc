 #include "open_leg.h"

namespace ai_server::model::motion {

open_leg::open_leg() : base(33) {}

std::tuple<double, double, double> open_leg::execute() {
  return std::make_tuple<double, double, double>(0.0, 0.0, 0.0);
}

} // namespace ai_server::model::motion