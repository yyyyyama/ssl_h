#ifndef AI_SERVER_MODEL_UPDATER_REFBOX_H
#define AI_SERVER_MODEL_UPDATER_REFBOX_H

#include <shared_mutex>
#include <Eigen/Geometry>
#include "ai_server/model/refbox.h"

// 前方宣言
namespace ssl_protos {
namespace gc {
class Referee;
}
} // namespace ssl_protos

namespace ai_server {
namespace model {
namespace updater {

/// @class   refbox
/// @brief   SSL RefBoxのRefereeパケットでRefBoxの情報を更新する
class refbox {
  mutable std::shared_timed_mutex mutex_;
  model::refbox refbox_;

  /// 変換行列
  Eigen::Affine3d affine_;

public:
  refbox();

  refbox(const refbox&) = delete;
  refbox& operator=(const refbox&) = delete;

  /// @brief          RefereeパケットでRefBoxの情報を更新する
  /// @param referee  SSL Referee BoxのRefereeパケット
  void update(const ssl_protos::gc::Referee& referee);

  /// @brief           updaterに変換行列を設定する
  /// @param matrix    変換行列
  void set_transformation_matrix(const Eigen::Affine3d& matrix);

  /// @brief          値を取得する
  model::refbox value() const;
};

} // namespace updater
} // namespace model
} // namespace ai_server

#endif // AI_SERVER_MODEL_UPDATER_REFBOX_H
