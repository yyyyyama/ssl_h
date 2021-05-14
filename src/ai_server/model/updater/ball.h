#ifndef AI_SERVER_MODEL_UPDATER_BALL_H
#define AI_SERVER_MODEL_UPDATER_BALL_H

#include <memory>
#include <mutex>
#include <optional>
#include <type_traits>
#include <unordered_map>

#include <Eigen/Geometry>

#include "ai_server/filter/base.h"
#include "ai_server/model/ball.h"
#include "ssl-protos/vision_detection.pb.h"

namespace ai_server {
namespace model {
namespace updater {

/// @class   ball
/// @brief   SSL-VisionのDetectionパケットでボールの情報を更新する
class ball {
  /// 更新タイミングがsameなFilterの型
  using filter_same_type = filter::base<model::ball, filter::timing::same>;
  /// 更新タイミングがmanualなFilterの型
  using filter_manual_type = filter::base<model::ball, filter::timing::manual>;

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

  /// @brief           更新タイミングがsameなFilterを設定する
  /// @param args      Filterの引数
  /// @return          初期化されたFilterへのポインタ
  template <class Filter, class... Args>
  std::weak_ptr<std::enable_if_t<std::is_base_of<filter_same_type, Filter>::value, Filter>>
  set_filter(Args&&... args) {
    std::unique_lock lock(mutex_);
    filter_manual_.reset();
    auto p       = std::make_shared<Filter>(std::forward<Args>(args)...);
    filter_same_ = p;
    return p;
  }

  /// @brief           更新タイミングがmanualなFilterを設定する
  /// @param args      Filterの引数
  /// @return          初期化されたFilterへのポインタ
  template <class Filter, class... Args>
  std::weak_ptr<std::enable_if_t<std::is_base_of<filter_manual_type, Filter>::value, Filter>>
  set_filter(Args&&... args) {
    std::unique_lock lock(mutex_);
    filter_same_.reset();
    auto p = std::make_shared<Filter>(
        mutex_,
        // 値を更新する関数オブジェクト
        [this](std::optional<model::ball> value) {
          // 値を持っていた場合のみ値の更新を行う
          if (value) {
            std::unique_lock lock(mutex_);
            ball_ = *value;
            ball_.set_is_lost(false);
          } else {
            ball_.set_is_lost(true);
          }
        },
        // 残りの引数
        std::forward<Args>(args)...);
    filter_manual_ = p;
    return p;
  }

private:
  mutable std::recursive_mutex mutex_;

  /// 最終的な値
  model::ball ball_;

  /// 各カメラで検出されたボールの生データ
  std::unordered_map<unsigned int, ssl_protos::vision::Ball> raw_balls_;

  /// 更新タイミングがsameなFilter
  std::shared_ptr<filter_same_type> filter_same_;
  /// 更新タイミングがmanualなFilter
  std::shared_ptr<filter_manual_type> filter_manual_;

  /// 変換行列
  Eigen::Affine3d affine_;
};

} // namespace updater
} // namespace model
} // namespace ai_server

#endif // AI_SERVER_MODEL_UPDATER_BALL_H
