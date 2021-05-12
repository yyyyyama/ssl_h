#include "ai_server/util/math/affine.h"
#include "world.h"
#include "ssl-protos/vision_wrapper.pb.h"

namespace ai_server {
namespace model {
namespace updater {

void world::update(const ssl_protos::vision::Packet& packet) {
  if (packet.has_detection()) {
    const auto& detection = packet.detection();

    // 無効化されたカメラは無視する
    if (!is_camera_enabled(detection.camera_id())) return;

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
  matrix_ = matrix;
  ball_.set_transformation_matrix(matrix);
  robots_blue_.set_transformation_matrix(matrix);
  robots_yellow_.set_transformation_matrix(matrix);
}

void world::set_transformation_matrix(double x, double y, double theta) {
  matrix_ = util::math::make_transformation_matrix(x, y, theta);
  set_transformation_matrix(matrix_);
}

Eigen::Affine3d world::transformation_matrix() {
  return matrix_;
}

void world::disable_camera(unsigned int id) {
  // idが登録されていなかったら追加する
  std::lock_guard lock{mutex_};
  disabled_camera_.insert(id);
}

void world::enable_camera(unsigned int id) {
  // idが登録されていたら解除する
  std::lock_guard lock{mutex_};
  disabled_camera_.erase(id);
}

bool world::is_camera_enabled(unsigned int id) const {
  std::lock_guard lock{mutex_};
  return disabled_camera_.find(id) == disabled_camera_.end();
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
