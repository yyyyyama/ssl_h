#ifndef AI_SERVER_MODEL_WORLD_UPDATER_ROBOT_IMPL_H
#define AI_SERVER_MODEL_WORLD_UPDATER_ROBOT_IMPL_H

#include <algorithm>

#include "ai_server/util/math/affine.h"
#include "ai_server/util/time.h"
#include "robot.h"

namespace ai_server {
namespace model {
namespace updater {

// robot<Color>::src_の特殊化
// 青チームのロボットはssl_protos::vision::Frame::robots_blue()で取得する
template <>
const robot<model::team_color::blue>::source_function_pointer_type
    robot<model::team_color::blue>::src_ = &ssl_protos::vision::Frame::robots_blue;

// 黄チームのロボットはssl_protos::vision::Frame::robots_yellow()で取得する
template <>
const robot<model::team_color::yellow>::source_function_pointer_type
    robot<model::team_color::yellow>::src_ = &ssl_protos::vision::Frame::robots_yellow;

template <model::team_color Color>
robot<Color>::robot() : affine_{Eigen::Translation3d{.0, .0, .0}} {}

template <model::team_color Color>
void robot<Color>::update(const ssl_protos::vision::Frame& detection) {
  std::unique_lock lock(mutex_);

  // カメラID
  const auto camera_id = detection.camera_id();
  // キャプチャされた時間
  const auto captured_time =
      std::chrono::system_clock::time_point{util::to_duration(detection.t_capture())};

  // 保持している生データを更新する
  raw_robots_[camera_id] = (detection.*src_)();

  // カメラIDをKeyに, 各カメラで検出されたロボットの情報を保持しているraw_robots_から
  // | Key(cam_id) | Value({Robot})           |
  // | ----------- | ------------------------ |
  // |           0 | {Robot(ID0), Robot(ID1)} |
  // |           1 | {Robot(ID1), Robot(ID2)} |
  // ロボットIDをKeyに,
  // 1つ以上のロボットの情報をカメラIDとともに保持するハッシュテーブルを作る
  // | Key(robo_id)  | Value({cam_id, Robot})                       |
  // | ------------- | -------------------------------------------- |
  // |             0 | {{cam_id, Robot(ID0)}}                       |
  // |             1 | {{cam_id, Robot(ID1)}, {cam_id, Robot(ID1)}} |
  // |             2 | {{cam_id, Robot(ID2)}}                       |
  const auto candidates = [this] {
    using robot_with_camera_id = std::tuple<unsigned int, raw_data_array_type::const_iterator>;
    std::unordered_multimap<unsigned int, robot_with_camera_id> table{};
    for (auto it1 = raw_robots_.cbegin(); it1 != raw_robots_.cend(); ++it1) {
      const auto& robots_list = it1->second;
      for (auto it2 = robots_list.cbegin(); it2 != robots_list.cend(); ++it2) {
        table.emplace(it2->robot_id(), std::forward_as_tuple(it1->first, it2));
      }
    }
    return table;
  }();

  // 作ったハッシュテーブルから, 各IDの最もconfidenceの高い要素を選択して値の更新を行う
  robots_list_type reliables{};
  for (auto it = candidates.cbegin(); it != candidates.cend();) {
    const auto robot_id = it->first;

    // IDがrobot_idのロボットの中で, 最もconfidenceの高い値を選択する
    const auto range    = candidates.equal_range(robot_id);
    const auto reliable = std::max_element(range.first, range.second, [](auto& a, auto& b) {
      return std::get<1>(a.second)->confidence() < std::get<1>(b.second)->confidence();
    });

    // その値が検出されたカメラIDとdetectionのカメラIDを比較
    if (std::get<0>(reliable->second) == camera_id) {
      // 一致していたら値の更新を行う
      // (現在のカメラで新たに検出された or
      // 現在のカメラで検出された値のほうがconfidenceが高かった)
      const auto value    = util::math::transform(affine_, [reliable] {
        const auto r = std::get<1>(reliable->second);
        return model::robot{r->x(), r->y(), r->orientation()};
      }());
      reliables[robot_id] = value;

      // 2つのFilterが設定されておらず, かつfilter_initializer_が設定されていたら
      // filter_initializer_でFilterを初期化する
      if (filter_initializer_ && !filters_same_.count(robot_id) &&
          !filters_manual_.count(robot_id)) {
        filters_same_[robot_id] = filter_initializer_();
      }

      if (auto f = filters_same_.find(robot_id); f != filters_same_.end()) {
        // `timing::same` なFilterが設定されていたらFilterを通した値を使う
        if (auto v = f->second->update(value, captured_time); v.has_value()) {
          robots_[robot_id] = std::move(*v);
        } else {
          robots_.erase(robot_id);
        }
      } else if (auto f = filters_manual_.find(robot_id); f != filters_manual_.end()) {
        // `timing::manual` なFilterが設定されていたら観測値を通知する
        f->second->set_raw_value(value, captured_time);
      } else {
        // Filterが登録されていない場合はそのままの値を使う
        robots_[robot_id] = value;
      }
    } else {
      // カメラIDが一致しないときは前の値を引き継ぐ
      // (現在のカメラで検出されたがconfidenceが低かった or 現在のカメラで検出されなかった)
      reliables[robot_id] = reliable_robots_.at(robot_id);
    }

    // イテレータを次のロボットIDの位置まで進める
    it = range.second;
  }

  // 最終的なデータのリストから, フィールド全体で検出されなかったIDを取り除く
  for (auto it = robots_.begin(); it != robots_.end();) {
    const auto id = it->first;
    if (reliables.count(id)) {
      ++it;
    } else {
      // Filter が設定されていたらロストしたことを通知する
      if (auto f = filters_same_.find(id); f != filters_same_.end()) {
        if (auto v = f->second->update(std::nullopt, captured_time); v.has_value()) {
          robots_[id] = std::move(*v);
          ++it;
        } else {
          it = robots_.erase(it);
        }
      } else if (auto f = filters_manual_.find(id); f != filters_manual_.end()) {
        f->second->set_raw_value(std::nullopt, captured_time);
        ++it;
      } else {
        it = robots_.erase(it);
      }
    }
  }

  reliable_robots_ = std::move(reliables);
}

template <model::team_color Color>
typename robot<Color>::robots_list_type robot<Color>::value() const {
  std::unique_lock lock(mutex_);
  return robots_;
}

template <model::team_color Color>
void robot<Color>::set_transformation_matrix(const Eigen::Affine3d& matrix) {
  std::unique_lock lock(mutex_);
  affine_ = matrix;
}

template <model::team_color Color>
void robot<Color>::clear_filter(unsigned int id) {
  std::unique_lock lock(mutex_);
  filters_same_.erase(id);
  filters_manual_.erase(id);
}

template <model::team_color Color>
void robot<Color>::clear_all_filters() {
  std::unique_lock lock(mutex_);
  filters_same_.clear();
  filters_manual_.clear();
}

template <model::team_color Color>
void robot<Color>::clear_default_filter() {
  std::unique_lock lock(mutex_);
  filter_initializer_ = decltype(filter_initializer_){};
}

// 必要なチームカラーで明示的なtemplateのインスタンス化を行う
// これにより, class templateを用いているが実装をソースに分離することができる
// http://en.cppreference.com/w/cpp/language/class_template#Explicit_instantiation
template class robot<model::team_color::blue>;
template class robot<model::team_color::yellow>;

} // namespace updater
} // namespace model
} // namespace ai_server

#endif // AI_SERVER_MODEL_WORLD_UPDATER_ROBOT_H
