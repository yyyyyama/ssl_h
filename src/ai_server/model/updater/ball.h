#ifndef AI_SERVER_MODEL_UPDATER_BALL_H
#define AI_SERVER_MODEL_UPDATER_BALL_H

#include <memory>
#include <shared_mutex>
#include <type_traits>
#include <unordered_map>

#include <experimental/optional>

#include <Eigen/Geometry>

#include "ai_server/filter/base.h"
#include "ai_server/model/ball.h"
#include "ssl-protos/vision/detection.pb.h"

namespace ai_server {
namespace model {
namespace updater {

/// @class   ball
/// @brief   SSL-VisionのDetectionパケットでボールの情報を更新する
class ball {
  /// 更新タイミングがon_updatedなFilterの型
  using on_updated_filter_type = filter::base<model::ball, filter::timing::on_updated>;
  /// 更新タイミングがmanualなFilterの型
  using manual_filter_type = filter::base<model::ball, filter::timing::manual>;

public:
  ball();

  ball(const ball&) = delete;
  ball& operator=(const ball&) = delete;

  /// @brief           Detectionパケットを処理し, ボールの情報を更新する
  /// @param detection SSL-VisionのDetectionパケット
  void update(const ssl_protos::vision::Frame& detection);

  /// @brief           値を取得する
  model::ball value() const;

  /// @brief           updaterに変換行列を設定する
  /// @param matrix    変換行列
  void set_transformation_matrix(const Eigen::Affine3d& matrix);

  /// @brief           設定されたFilterを解除する
  void clear_filter();

  /// @brief           更新タイミングがon_updatedなFilterを設定する
  /// @param args      Filterの引数
  /// @return          初期化されたFilterへのポインタ
  template <class Filter, class... Args>
  std::weak_ptr<
      std::enable_if_t<std::is_base_of<on_updated_filter_type, Filter>::value, Filter>>
  set_filter(Args&&... args) {
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    manual_filter_.reset();
    auto p             = std::make_shared<Filter>(std::forward<Args>(args)...);
    on_updated_filter_ = p;
    return p;
  }

  /// @brief           更新タイミングがmanualなFilterを設定する
  /// @param args      Filterの引数
  /// @return          初期化されたFilterへのポインタ
  template <class Filter, class... Args>
  std::weak_ptr<std::enable_if_t<std::is_base_of<manual_filter_type, Filter>::value, Filter>>
  set_filter(Args&&... args) {
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    on_updated_filter_.reset();
    auto p = std::make_shared<Filter>(
        // 最新の値を取得する関数オブジェクト
        [this] {
          std::shared_lock<std::shared_timed_mutex> lock(mutex_);
          return reliable_ball_;
        },
        // 値を更新する関数オブジェクト
        [this](std::experimental::optional<model::ball> value) {
          // 現時点ではボールが存在しない場合を想定していないので,
          // 値を持っていた場合のみ値の更新を行う
          if (value) {
            std::unique_lock<std::shared_timed_mutex> lock(mutex_);
            ball_ = *value;
          }
        },
        // 残りの引数
        std::forward<Args>(args)...);
    manual_filter_ = p;
    return p;
  }

private:
  mutable std::shared_timed_mutex mutex_;

  /// 最終的な値
  model::ball ball_;

  /// 各カメラで検出されたボールの生データ
  std::unordered_map<unsigned int, ssl_protos::vision::Ball> raw_balls_;
  /// 検出された中から選ばれた, 最も確かとされる値
  std::experimental::optional<model::ball> reliable_ball_;

  /// 更新タイミングがon_updatedなFilter
  std::shared_ptr<on_updated_filter_type> on_updated_filter_;
  /// 更新タイミングがmanualなFilter
  std::shared_ptr<manual_filter_type> manual_filter_;

  /// 変換行列
  Eigen::Affine3d affine_;
};

} // namespace updater
} // namespace model
} // namespace ai_server

#endif // AI_SERVER_MODEL_UPDATER_BALL_H
