#include <cmath>

#include "field.h"
#include "ssl-protos/vision/geometry.pb.h"

namespace ai_server {
namespace model {
namespace updater {

field::field() : field_{} {}

void field::update(const ssl_protos::vision::Geometry& geometry) {
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);

  const auto& f = geometry.field();

  field_.set_length(f.field_length());
  field_.set_width(f.field_width());
  field_.set_goal_width(f.goal_width());

  for (const auto& arc : f.field_arcs()) {
    if (arc.name() == "CenterCircle") {
      field_.set_center_radius(arc.radius());
    }
  }
}

model::field field::value() const {
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  return field_;
}

} // namespace updater
} // namespace model
} // namespace ai_server
