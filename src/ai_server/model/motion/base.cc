#include "base.h"

namespace ai_server::model::motion {

base::base(const std::string& name) : name_(name) {}

std::string base::name() {
  return name_;
}

} // namespace ai_server::model::motion
