#include "run_forward.h"

namespace ai_server::model::motion {

run_forward::run_forward() : base(22) {}

std::tuple<double, double, double> run_forward::execute() {
  return std::make_tuple<double, double, double>(150.0, 0.0, 0.0);
}

} // namespace ai_server::model::motion