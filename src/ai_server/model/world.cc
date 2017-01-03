#include <cmath>
#include <queue>
#include <vector>

#include "world.h"
#include "detail/confidence_comparator.h"

namespace ai_server {
namespace model {

model::field world::field() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return field_;
}

model::ball world::ball() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return ball_;
}

std::unordered_map<unsigned int, model::robot> world::robots_blue() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return robots_blue_;
}

std::unordered_map<unsigned int, model::robot> world::robots_yellow() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return robots_yellow_;
}

void world::update(const ssl_protos::vision::Packet& packet) {
  if (packet.has_detection()) {
    process_packet(packet.detection());
  }

  if (packet.has_geometry()) {
    process_packet(packet.geometry());
  }
}

void world::process_packet(const ssl_protos::vision::Frame& detection) {
  using namespace ssl_protos::vision;

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
  std::lock_guard<std::mutex> lock(mutex_);

  // TODO:
  // 現在の実装は, フィールドにボールが1つしかないと仮定し,
  // 検出されたconfidenceの最も高いものを選んでいる.
  // 今後, フィールドにボールが複数存在する場合の処理を検討する必要がある.
  // https://github.com/kiksworks/ai-server/projects/1#card-1115932
  while (!bq.empty()) {
    const auto& top = bq.top();

    ball_ = {top.x(), top.y(), top.z()};
    bq.pop();
  }

  robots_blue_.clear();
  while (!rbq.empty()) {
    const auto& top = rbq.top();

    robots_blue_[top.robot_id()] = {top.robot_id(), top.x(), top.y()};
    robots_blue_[top.robot_id()].set_theta(top.orientation());
    rbq.pop();
  }

  robots_yellow_.clear();
  while (!ryq.empty()) {
    const auto& top = ryq.top();

    robots_yellow_[top.robot_id()] = {top.robot_id(), top.x(), top.y()};
    robots_yellow_[top.robot_id()].set_theta(top.orientation());
    ryq.pop();
  }
}

void world::process_packet(const ssl_protos::vision::Geometry& geometry) {
  std::lock_guard<std::mutex> lock(mutex_);

  const auto& f = geometry.field();
  field_.set_length(f.field_length());
  field_.set_width(f.field_width());
  field_.set_goal_width(f.goal_width());

  for (const auto& arc : f.field_arcs()) {
    if (arc.name() == "CenterCircle") {
      field_.set_center_radius(arc.radius());
    } else if (arc.name() == "LeftFieldLeftPenaltyArc") {
      field_.set_penalty_radius(arc.radius());
    }
  }

  for (const auto& line : f.field_lines()) {
    if (line.name() == "LeftPenaltyStretch") {
      field_.set_penalty_line_length(
          std::abs(std::hypotf(line.p2().x() - line.p1().x(), line.p2().y() - line.p1().y())));
    }
  }
}

} // namespace model
} // namespace ai_server
