#ifndef AI_SERVER_MODEL_UPDATER_ROBOT_H
#define AI_SERVER_MODEL_UPDATER_ROBOT_H

#include <functional>
#include <memory>
#include <mutex>
#include <tuple>
#include <type_traits>
#include <unordered_map>

#include <Eigen/Geometry>

#include "ai_server/filter/base.h"
#include "ai_server/model/robot.h"
#include "ai_server/model/team_color.h"
#include "ssl-protos/vision_detection.pb.h"

namespace ai_server {
namespace model {
namespace updater {

/// @class   robot
/// @brief   SSL-VisionのDetectionパケットでロボットの情報を更新する
template <model::team_color Color>
class robot {
  /// KeyがID, Valueがロボットのハッシュテーブルの型
  using robots_list_type = std::unordered_map<unsigned int, model::robot>;

  /// 生データの型
  using raw_data_type = ssl_protos::vision::Robot;
  /// 生データが格納されている配列の型
  using raw_data_array_type = google::protobuf::RepeatedPtrField<raw_data_type>;
  /// ロボットの生データを取得するFrameのメンバ関数へのポインタの型
  using source_function_pointer_type =
      const raw_data_array_type& (ssl_protos::vision::Frame::*)() const;

  /// 更新タイミングがsameなFilterの型
  using filters_same_type = filter::base<model::robot, filter::timing::same>;
  /// 更新タイミングがmanualなFilterの型
  using filters_manual_type = filter::base<model::robot, filter::timing::manual>;

public:
  robot();
  robot(const robot&) = delete;
  robot& operator=(const robot&) = delete;

  /// @brief           Detectionパケットを処理し, ボールの情報を更新する
  /// @param detection SSL-VisionのDetectionパケット
  void update(const ssl_protos::vision::Frame& detection);

  /// @brief           値を取得する
  robots_list_type value() const;

  /// @brief           updaterに変換行列を設定する
  /// @param matrix    変換行列
  void set_transformation_matrix(const Eigen::Affine3d& matrix);

  /// @brief           設定されたFilterを解除する
  /// @param id        Filterを解除するロボットのID
  void clear_filter(unsigned int id);

  /// @brief           設定された全てのFilterを解除する
  void clear_all_filters();

  /// @brief           設定されたデフォルトのFilterを解除する
  ///
  /// あくまでfilter_initializer_を空にするだけなので,
  /// 既にfilter_initializer_によって初期化されたものを解除したい場合は
  /// clear_filter()などを呼ぶ必要がある
  void clear_default_filter();

  /// @brief           更新タイミングがsameなFilterを設定する
  /// @param id        Filterを設定するロボットのID
  /// @param args      Filterの引数
  /// @return          初期化されたFilterへのポインタ
  template <class Filter, class... Args>
  std::weak_ptr<std::enable_if_t<std::is_base_of<filters_same_type, Filter>::value, Filter>>
  set_filter(unsigned int id, Args&&... args) {
    std::unique_lock lock(mutex_);
    filters_manual_.erase(id);
    auto p            = std::make_shared<Filter>(std::forward<Args>(args)...);
    filters_same_[id] = p;
    return p;
  }

  /// @brief           更新タイミングがmanualなFilterを設定する
  /// @param id        Filterを設定するロボットのID
  /// @param args      Filterの引数
  /// @return          初期化されたFilterへのポインタ
  template <class Filter, class... Args>
  std::weak_ptr<std::enable_if_t<std::is_base_of<filters_manual_type, Filter>::value, Filter>>
  set_filter(unsigned int id, Args&&... args) {
    std::unique_lock lock(mutex_);
    filters_same_.erase(id);
    auto p = std::make_shared<Filter>(
        mutex_,
        // 値を更新する関数オブジェクト
        [id, this](std::optional<model::robot> value) {
          std::unique_lock lock(mutex_);
          if (value) {
            // valueが値を持っていた場合はその値で更新
            robots_[id] = *value;
          } else {
            // valueが値を持っていなかった場合はリストから要素を削除する
            robots_.erase(id);
          }
        },
        // 残りの引数
        std::forward<Args>(args)...);
    filters_manual_[id] = p;
    return p;
  }

  /// @brief           デフォルトのFilterを設定する
  /// @param args      Filterの引数
  ///
  /// set_filter()でFilterが設定されていないロボットが使うFilterを設定する
  /// ここで設定できるものは更新タイミングがsameなFilterのみとする
  template <class Filter, class... Args>
  auto set_default_filter(Args... args)
      -> std::enable_if_t<std::is_base_of<filters_same_type, Filter>::value> {
    std::unique_lock lock(mutex_);
    filter_initializer_ = [args...] { return std::make_shared<Filter>(args...); };
  }

private:
  mutable std::recursive_mutex mutex_;

  /// ロボットの生データを取得するFrameのメンバ関数へのポインタ
  /// このポインタを各チームカラーに対して特殊化することで, 同じ更新処理を使えるようにしている
  static const source_function_pointer_type src_;

  /// 最終的な値
  robots_list_type robots_;

  /// 各カメラで検出されたロボットの生データ (KeyはカメラID)
  std::unordered_map<unsigned int, raw_data_array_type> raw_robots_;
  /// 検出された中から選ばれた, 各IDをの最も確かとされる値のリスト
  robots_list_type reliable_robots_;

  /// 更新タイミングがsameなFilter
  std::unordered_map<unsigned int, std::shared_ptr<filters_same_type>> filters_same_;
  /// 更新タイミングがmanualなFilter
  std::unordered_map<unsigned int, std::shared_ptr<filters_manual_type>> filters_manual_;
  /// Filterを初期化するための関数オブジェクト
  std::function<std::shared_ptr<filters_same_type>()> filter_initializer_;

  /// 変換行列
  Eigen::Affine3d affine_;
};

} // namespace updater
} // namespace model
} // namespace ai_server

#endif // AI_SERVER_MODEL_UPDATER_ROBOT_H
