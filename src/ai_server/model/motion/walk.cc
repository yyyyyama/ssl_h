#include "walk.h"

namespace ai_server::model::motion {

walk::walk() : base(5) {}

std::tuple<double, double, double> walk::execute() {
  return std::make_tuple<double, double, double>(100.0, 0.0, 0.0);
}

} // namespace ai_server::model::motion