#include "ai_server/util/math/affine.h"
#include "world.h"
#include "ssl-protos/vision/wrapperpacket.pb.h"

namespace ai_server {
namespace model {
namespace updater {

void world::update(const ssl_protos::vision::Packet& packet) {
  if (packet.has_detection()) {
    const auto& detection = packet.detection();

    // 無効化されたカメラは無視する
    if (std::find(disabled_camera_.cbegin(), disabled_camera_.cend(), detection.camera_id()) !=
        disabled_camera_.cend()) {
      return;
    }

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

void world::set_transformation_matrix(const Eigen::Affine3d& matrix) {
  ball_.set_transformation_matrix(matrix);
  robots_blue_.set_transformation_matrix(matrix);
  robots_yellow_.set_transformation_matrix(matrix);
}

void world::set_transformation_matrix(double x, double y, double theta) {
  set_transformation_matrix(util::math::make_transformation_matrix(x, y, theta));
}

void world::disable_camera(unsigned int id) {
  // idが登録されていなかったら追加する
  if (std::find(disabled_camera_.cbegin(), disabled_camera_.cend(), id) ==
      disabled_camera_.cend()) {
    disabled_camera_.push_back(id);
  }
}

void world::enable_camera(unsigned int id) {
  // idが登録されていたら解除する
  auto it = std::find(disabled_camera_.begin(), disabled_camera_.end(), id);
  if (it != disabled_camera_.end()) {
    disabled_camera_.erase(it);
  }
}

bool world::is_camera_enabled(unsigned int id) const {
  return std::find(disabled_camera_.cbegin(), disabled_camera_.cend(), id) ==
         disabled_camera_.cend();
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
