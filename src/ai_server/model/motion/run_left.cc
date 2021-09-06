#include "run_left.h"

namespace ai_server::model::motion {

run_left::run_left() : base(520) {}

std::tuple<double, double, double> run_left::execute() {
  return std::make_tuple<double, double, double>(0.0, -150.0, 0.0);
}

} // namespace ai_server::model::motion