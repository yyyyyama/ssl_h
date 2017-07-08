#include "world.h"
#include "ssl-protos/vision/wrapperpacket.pb.h"

namespace ai_server {
namespace model {
namespace updater {

void world::update(const ssl_protos::vision::Packet& packet) {
  if (packet.has_detection()) {
    const auto& detection = packet.detection();
    ball_.update(detection);
    robots_blue_.update(detection);
    robots_yellow_.update(detection);
  }

  if (packet.has_geometry()) {
    const auto& geometry = packet.geometry();
    field_.update(geometry);
  }
}

model::world world::value() const {
  return {field_.value(), ball_.value(), robots_blue_.value(), robots_yellow_.value()};
}

field& world::field_updater() {
  return field_;
}

ball& world::ball_updater() {
  return ball_;
}

robot<model::team_color::blue>& world::robots_blue_updater() {
  return robots_blue_;
}

robot<model::team_color::yellow>& world::robots_yellow_updater() {
  return robots_yellow_;
}

} // namespace updater
} // namespace model
} // namespace ai_server
