#include "run_right.h"

namespace ai_server::model::motion {

run_right::run_right() : base(25) {}

std::tuple<double, double, double> run_right::execute() {
  return std::make_tuple<double, double, double>(0.0, -150.0, 0.0);
}

} // namespace ai_server::model::motion