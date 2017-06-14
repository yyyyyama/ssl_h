#include <cmath>

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
  std::lock_guard<std::mutex> lock(mutex_);

  // カメラIDを取得
  const auto& camera_id = detection.camera_id();

  // キャプチャされた時間を取得
  const auto captured_time =
      std::chrono::high_resolution_clock::time_point{std::chrono::microseconds{
          static_cast<std::chrono::microseconds::rep>(detection.t_capture() * 1e6)}};

  // 取得したIDのdetectionパケットを更新
  detection_packets_[camera_id] = detection;

  std::vector<ball_with_camera_id> balls{};
  robots_table robots_blue_table{};
  robots_table robots_yellow_table{};

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

  world_.set_ball(build_ball_data(camera_id, captured_time, balls, world_.ball()));
  world_.set_robots_blue(build_robots_list(false, camera_id, captured_time, robots_blue_table,
                                           world_.robots_blue()));
  world_.set_robots_yellow(build_robots_list(true, camera_id, captured_time,
                                             robots_yellow_table, world_.robots_yellow()));
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

model::ball world_updater::build_ball_data(
    unsigned int camera_id, std::chrono::high_resolution_clock::time_point captured_time,
    const std::vector<ball_with_camera_id>& balls, const model::ball& prev_data) {
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

  // フィールドにボールがあれば情報を更新する
  if (reliable_element != balls.cend()) {
    // 各カメラで2つ以上のボールが映らないものとする
    // 選択したデータのカメラIDとdetectionのカメラIDが一致していたらデータを更新する
    if (std::get<0>(*reliable_element) == camera_id) {
      const auto& ball_data = std::get<1>(*reliable_element);
      ret.set_x(ball_data->x());
      ret.set_y(ball_data->y());
      ret.set_z(ball_data->z());
    }
  }

  return ret;
}

void world_updater::add_robots_to_table(
    robots_table& table, unsigned int camera_id,
    const google::protobuf::RepeatedPtrField<ssl_protos::vision::Robot>& robots) {
  for (auto it = robots.cbegin(); it != robots.cend(); ++it) {
    table.emplace(it->robot_id(), std::forward_as_tuple(camera_id, it));
  }
}

model::world::robots_list world_updater::build_robots_list(
    bool is_yellow, unsigned int camera_id,
    std::chrono::high_resolution_clock::time_point captured_time, const robots_table& table,
    const model::world::robots_list& prev_data) {
  model::world::robots_list ret{};

  for (auto it = table.cbegin(); it != table.cend();) {
    const auto robot_id = it->first;

    // tableに格納されたIDがrobot_idのロボットの中で, 最もconfidenceの高い要素を選択する
    const auto range = table.equal_range(robot_id);
    const auto reliable_element =
        std::max_element(range.first, range.second, [](auto& a, auto& b) {
          return std::get<1>(a.second)->confidence() < std::get<1>(b.second)->confidence();
        });

    const auto& robot_data = std::get<1>(reliable_element->second);

    // 前のデータにIDがrobot_idの要素があればそれをコピーし,
    // なければ新たに作成する
    ret.emplace(robot_id, prev_data.count(robot_id)
                              ? prev_data.at(robot_id)
                              : model::robot{robot_id, robot_data->x(), robot_data->y(),
                                             robot_data->orientation()});

    // 選択したデータのカメラIDとdetectionのカメラIDが一致したら情報を更新する
    if (std::get<0>(reliable_element->second) == camera_id) {
      ret[robot_id].set_x(robot_data->x());
      ret[robot_id].set_y(robot_data->y());
      ret[robot_id].set_theta(robot_data->orientation());
    }

    // イテレータを次のロボットIDの位置まで進める
    it = range.second;
  }

  return ret;
}

} // namespace model
} // namespace ai_server
