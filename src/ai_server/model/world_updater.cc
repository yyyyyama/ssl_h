#include <cmath>
#include <queue>
#include <vector>

#include "ai_server/util/algorithm.h"
#include "detail/confidence_comparator.h"
#include "world_updater.h"

namespace ai_server {
namespace model {

const model::world& world_updater::world_model() const {
  return world_;
}

void world_updater::update(const ssl_protos::vision::Packet& packet) {
  if (packet.has_detection()) {
    process_packet(packet.detection());
  }

  if (packet.has_geometry()) {
    process_packet(packet.geometry());
  }
}

void world_updater::process_packet(const ssl_protos::vision::Frame& detection) {
  using namespace ssl_protos::vision;

  std::lock_guard<std::mutex> lock(mutex_);

  // 取得したIDのdetectionパケットを更新
  detection_packets_[detection.camera_id()] = detection;

  // Ball::confidence()が低い順にソートされるpriority_queue
  using ball_queue =
      std::priority_queue<Ball, std::vector<Ball>, decltype(detail::confidence_comparator)>;
  ball_queue bq(detail::confidence_comparator);

  // Robot::confidence()が低い順にソートされるpriority_queue
  using robot_queue =
      std::priority_queue<Robot, std::vector<Robot>, decltype(detail::confidence_comparator)>;
  robot_queue rbq(detail::confidence_comparator); // robots_blue
  robot_queue ryq(detail::confidence_comparator); // robots_yellow

  // 保持しているパケットに含まれるball, robotを全てキューに追加
  for (const auto& dp : detection_packets_) {
    for (auto&& b : dp.second.balls()) {
      bq.emplace(b);
    }
    for (auto&& r : dp.second.robots_blue()) {
      rbq.emplace(r);
    }
    for (auto&& r : dp.second.robots_yellow()) {
      ryq.emplace(r);
    }
  }

  // 内部データを更新する
  // 更新に使うキューはconfidenceの低い順にソートされているので,
  // IDが重複している場合はconfidenceの高いもので上書きされる
  using ai_server::util::pop_each;

  // TODO:
  // 現在の実装は, フィールドにボールが1つしかないと仮定し,
  // 検出されたconfidenceの最も高いものを選んでいる.
  // 今後, フィールドにボールが複数存在する場合の処理を検討する必要がある.
  // https://github.com/kiksworks/ai-server/projects/1#card-1115932
  model::ball ball{};
  pop_each(bq, [&ball](auto&& top) { ball = {top.x(), top.y(), top.z()}; });

  std::unordered_map<unsigned int, model::robot> robots_blue{};
  pop_each(rbq, [&robots_blue](auto&& top) {
    robots_blue[top.robot_id()] = {top.robot_id(), top.x(), top.y(), top.orientation()};
  });

  std::unordered_map<unsigned int, model::robot> robots_yellow{};
  pop_each(ryq, [&robots_yellow](auto&& top) {
    robots_yellow[top.robot_id()] = {top.robot_id(), top.x(), top.y(), top.orientation()};
  });

  world_.set_ball(std::move(ball));
  world_.set_robots_blue(std::move(robots_blue));
  world_.set_robots_yellow(std::move(robots_yellow));
}

void world_updater::process_packet(const ssl_protos::vision::Geometry& geometry) {
  const auto& f = geometry.field();
  model::field field{};

  field.set_length(f.field_length());
  field.set_width(f.field_width());
  field.set_goal_width(f.goal_width());

  for (const auto& arc : f.field_arcs()) {
    if (arc.name() == "CenterCircle") {
      field.set_center_radius(arc.radius());
    } else if (arc.name() == "LeftFieldLeftPenaltyArc") {
      field.set_penalty_radius(arc.radius());
    }
  }

  for (const auto& line : f.field_lines()) {
    if (line.name() == "LeftPenaltyStretch") {
      field.set_penalty_line_length(
          std::abs(std::hypotf(line.p2().x() - line.p1().x(), line.p2().y() - line.p1().y())));
    }
  }

  world_.set_field(std::move(field));
}

} // namespace model
} // namespace ai_server
