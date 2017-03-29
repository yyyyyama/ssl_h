#include <algorithm>
#include <cmath>
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

  std::vector<ball_with_camera_id_t> balls{};
  robots_table_t robots_blue_table{};
  robots_table_t robots_yellow_table{};

  for (const auto& dp : detection_packets_) {
    const auto& d = dp.second;

    for (auto it = d.balls().cbegin(); it != d.balls().cend(); ++it) {
      balls.emplace_back(d.camera_id(), it);
    }

    // カメラIDをKeyに, 各カメラの最新のパケットを保持しているdetection_packets_から
    // | Key(cam_id) | Value(Packet)                    |
    // | ----------- | -------------------------------- |
    // |           0 | Packet({Robot(ID0), Robot(ID1)}) |
    // |           1 | Packet({Robot(ID1), Robot(ID2)}) |
    // ロボットIDをKeyに,
    // 1つ以上のロボットの情報をカメラIDとともに保持するハッシュテーブルを作る
    // | Key(robo_id)  | Value({cam_id, Robot})                       |
    // | ------------- | -------------------------------------------- |
    // |             0 | {{cam_id, Robot(ID0)}}                       |
    // |             1 | {{cam_id, Robot(ID1)}, {cam_id, Robot(ID1)}} |
    // |             2 | {{cam_id, Robot(ID2)}}                       |
    add_robots_to_table(robots_blue_table, d.camera_id(), d.robots_blue());
    add_robots_to_table(robots_yellow_table, d.camera_id(), d.robots_yellow());
  }

  world_.set_ball(build_ball_data(balls, world_.ball()));
  world_.set_robots_blue(build_robot_list(robots_blue_table, world_.robots_blue()));
  world_.set_robots_yellow(build_robot_list(robots_yellow_table, world_.robots_yellow()));
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

model::ball world_updater::build_ball_data(const std::vector<ball_with_camera_id_t>& balls,
                                           const model::ball& prev_data) {
  auto ret = prev_data;

  // TODO:
  // 現在の実装は, フィールドにボールが1つしかないと仮定し,
  // 検出されたconfidenceの最も高いものを選んでいる.
  // 今後, フィールドにボールが複数存在する場合の処理を検討する必要がある.
  // https://github.com/kiksworks/ai-server/projects/1#card-1115932
  const auto reliable_element =
      std::max_element(balls.cbegin(), balls.cend(), [](auto& a, auto& b) {
        return std::get<1>(a)->confidence() < std::get<1>(b)->confidence();
      });

  const auto& ball_data = std::get<1>(*reliable_element);
  ret.set_x(ball_data->x());
  ret.set_y(ball_data->y());
  ret.set_z(ball_data->z());

  return ret;
}

void world_updater::add_robots_to_table(
    robots_table_t& table, unsigned int camera_id,
    const google::protobuf::RepeatedPtrField<ssl_protos::vision::Robot>& robots) {
  for (auto it = robots.cbegin(); it != robots.cend(); ++it) {
    table.emplace(it->robot_id(), std::forward_as_tuple(camera_id, it));
  }
}

std::unordered_map<unsigned int, model::robot> world_updater::build_robot_list(
    const robots_table_t& table,
    const std::unordered_map<unsigned int, model::robot>& prev_data) {
  std::unordered_map<unsigned int, model::robot> ret{};

  for (auto it = table.cbegin(); it != table.cend();) {
    const auto robot_id = it->first;

    // tableに格納されたIDがrobot_idのロボットの中で, 最もconfidenceの高い要素を選択する
    const auto range = table.equal_range(robot_id);
    const auto reliable_element =
        std::max_element(range.first, range.second, [](auto& a, auto& b) {
          return std::get<1>(a.second)->confidence() < std::get<1>(b.second)->confidence();
        });

    const auto& robot_data = std::get<1>(reliable_element->second);
    if (prev_data.count(robot_id)) {
      // 前のデータにIDがrobot_idの要素があれば, それをコピーしてから各情報を更新する
      ret.emplace(robot_id, prev_data.at(robot_id));
      ret[robot_id].set_x(robot_data->x());
      ret[robot_id].set_y(robot_data->y());
      ret[robot_id].set_theta(robot_data->orientation());
    } else {
      // なければ新たに要素を作成する
      ret.emplace(robot_id, model::robot{robot_id, robot_data->x(), robot_data->y(),
                                         robot_data->orientation()});
    }

    // イテレータを次のロボットIDの位置まで進める
    it = range.second;
  }

  return ret;
}

} // namespace model
} // namespace ai_server
