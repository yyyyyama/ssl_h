#include <algorithm>
#include <chrono>

#include "ai_server/util/math/affine.h"
#include "ai_server/util/time.h"
#include "ball.h"

namespace ai_server {
namespace model {
namespace updater {

ball::ball() : ball_{}, affine_{Eigen::Translation3d{.0, .0, .0}} {}

model::ball ball::value() const {
  std::unique_lock lock(mutex_);
  return ball_;
}

void ball::set_transformation_matrix(const Eigen::Affine3d& matrix) {
  std::unique_lock lock(mutex_);
  affine_ = matrix;
}

void ball::update(const ssl_protos::vision::Frame& detection) {
  std::unique_lock lock(mutex_);

  // カメラID
  const auto& camera_id = detection.camera_id();
  // キャプチャされた時間
  const auto captured_time =
      std::chrono::system_clock::time_point{util::to_duration(detection.t_capture())};

  // 検出されたボールの中から, 最もconfidenceの高い値を選択候補に登録する
  // FIXME:
  // 現在の実装は, フィールドにボールが1つしかないと仮定している
  // 1つのカメラで複数のボールが検出された場合, 意図しないデータが選択される可能性がある
  const auto& balls    = detection.balls();
  const auto candidate = std::max_element(balls.cbegin(), balls.cend(), [](auto& a, auto& b) {
    return a.confidence() < b.confidence();
  });
  if (candidate != balls.cend()) {
    raw_balls_[camera_id] = *candidate;
  } else {
    raw_balls_.erase(camera_id);
  }

  // 候補の中から, 最もconfidenceの高いボールを求める
  const auto reliable =
      std::max_element(raw_balls_.cbegin(), raw_balls_.cend(), [](auto& a, auto& b) {
        return std::get<1>(a).confidence() < std::get<1>(b).confidence();
      });

  if (reliable != raw_balls_.cend()) {
    // 選択された値のカメラIDとdetectionのカメラIDが一致していたらデータを更新する
    if (std::get<0>(*reliable) == camera_id) {
      const auto value = util::math::transform(affine_, [reliable] {
        const auto& r = std::get<1>(*reliable);
        return model::ball{r.x(), r.y(), r.z()};
      }());

      if (filter_same_) {
        // filter_same_が設定されていたらFilterを通した値を使う
        if (auto v = filter_same_->update(value, captured_time); v.has_value()) {
          ball_ = std::move(*v);
          ball_.set_is_lost(false);
        } else {
          ball_.set_is_lost(true);
        }
      } else if (filter_manual_) {
        // filter_manual_が設定されていたら観測値を通知する
        filter_manual_->set_raw_value(value, captured_time);
      } else {
        // Filterが登録されていない場合はそのままの値を使う
        ball_.set_x(value.x());
        ball_.set_y(value.y());
        ball_.set_z(value.z());
        ball_.set_is_lost(false);
      }
    }
  } else {
    // Filter が設定されていたらロストしたことを通知する
    if (filter_same_) {
      if (auto v = filter_same_->update(std::nullopt, captured_time); v.has_value()) {
        ball_ = std::move(*v);
        ball_.set_is_lost(false);
      } else {
        ball_.set_is_lost(true);
      }
    } else if (filter_manual_) {
      filter_manual_->set_raw_value(std::nullopt, captured_time);
    } else {
      ball_.set_is_lost(true);
    }
  }
}

void ball::clear_filter() {
  std::unique_lock lock(mutex_);
  filter_same_.reset();
  filter_manual_.reset();
}

} // namespace updater
} // namespace model
} // namespace ai_server
