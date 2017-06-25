#include <algorithm>
#include <chrono>

#include "ball.h"

namespace ai_server {
namespace model {
namespace updater {

ball::ball() : ball_{} {}

model::ball ball::value() const {
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  return ball_;
}

void ball::update(const ssl_protos::vision::Frame& detection) {
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);

  // カメラID
  const auto& camera_id = detection.camera_id();
  // キャプチャされた時間
  const auto captured_time =
      std::chrono::high_resolution_clock::time_point{std::chrono::microseconds{
          static_cast<std::chrono::microseconds::rep>(detection.t_capture() * 1e6)}};

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
      const auto& value = std::get<1>(*reliable);
      reliable_ball_    = model::ball{value.x(), value.y(), value.z()};

      if (on_updated_filter_) {
        // on_updated_filter_が設定されていたらFilterを通した値を使う
        ball_ = on_updated_filter_->update(*reliable_ball_, captured_time);
      } else if (!manual_filter_) {
        // Filterが登録されていない場合はそのままの値を使う
        ball_.set_x(reliable_ball_->x());
        ball_.set_y(reliable_ball_->y());
        ball_.set_z(reliable_ball_->z());
      }
    }
  } else {
    // 現時点ではボールが存在しない場合を想定していないので何もしない
    reliable_ball_ = std::experimental::nullopt;
  }
}

void ball::clear_filter() {
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);
  on_updated_filter_.reset();
  manual_filter_.reset();
}

} // namespace updater
} // namespace model
} // namespace ai_server
