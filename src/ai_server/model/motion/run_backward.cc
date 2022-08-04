#include "run_backward.h"

namespace ai_server::model::motion {

run_backward::run_backward() : base(23) {}

std::tuple<double, double, double> run_backward::execute() {
  return std::make_tuple<double, double, double>(-150.0, 0.0, 0.0);
}

} // namespace ai_server::model::motion