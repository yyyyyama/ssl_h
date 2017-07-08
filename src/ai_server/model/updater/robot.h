#ifndef AI_SERVER_MODEL_UPDATER_ROBOT_H
#define AI_SERVER_MODEL_UPDATER_ROBOT_H

#include <functional>
#include <memory>
#include <shared_mutex>
#include <tuple>
#include <type_traits>
#include <unordered_map>

#include "ai_server/filter/base.h"
#include "ai_server/model/robot.h"
#include "ai_server/model/team_color.h"
#include "ssl-protos/vision/detection.pb.h"

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

  /// 更新タイミングがon_updatedなFilterの型
  using on_updated_filter_type = filter::base<model::robot, filter::timing::on_updated>;
  /// 更新タイミングがmanualなFilterの型
  using manual_filter_type = filter::base<model::robot, filter::timing::manual>;

public:
  /// @brief           Detectionパケットを処理し, ボールの情報を更新する
  /// @param detection SSL-VisionのDetectionパケット
  void update(const ssl_protos::vision::Frame& detection);

  /// @brief           値を取得する
  robots_list_type value() const;

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

  /// @brief           更新タイミングがon_updatedなFilterを設定する
  /// @param id        Filterを設定するロボットのID
  /// @param args      Filterの引数
  /// @return          初期化されたFilterへのポインタ
  template <class Filter, class... Args>
  std::weak_ptr<
      std::enable_if_t<std::is_base_of<on_updated_filter_type, Filter>::value, Filter>>
  set_filter(unsigned int id, Args&&... args) {
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    manual_filters_.erase(id);
    auto p                  = std::make_shared<Filter>(std::forward<Args>(args)...);
    on_updated_filters_[id] = p;
    return p;
  }

  /// @brief           更新タイミングがmanualなFilterを設定する
  /// @param id        Filterを設定するロボットのID
  /// @param args      Filterの引数
  /// @return          初期化されたFilterへのポインタ
  template <class Filter, class... Args>
  std::weak_ptr<std::enable_if_t<std::is_base_of<manual_filter_type, Filter>::value, Filter>>
  set_filter(unsigned int id, Args&&... args) {
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    on_updated_filters_.erase(id);
    auto p = std::make_shared<Filter>(
        // 最新の値を取得する関数オブジェクト
        [id, this]() -> std::experimental::optional<model::robot> {
          std::shared_lock<std::shared_timed_mutex> lock(mutex_);
          // 選ばれた観測データの中に対象のロボットの値があればそれを, なければnulloptを返す
          if (reliable_robots_.count(id)) {
            return reliable_robots_.at(id);
          } else {
            return std::experimental::nullopt;
          }
        },
        // 値を更新する関数オブジェクト
        [id, this](std::experimental::optional<model::robot> value) {
          std::unique_lock<std::shared_timed_mutex> lock(mutex_);
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
    manual_filters_[id] = p;
    return p;
  }

  /// @brief           デフォルトのFilterを設定する
  /// @param args      Filterの引数
  ///
  /// set_filter()でFilterが設定されていないロボットが使うFilterを設定する
  /// ここで設定できるものは更新タイミングがon_updatedなFilterのみとする
  template <class Filter, class... Args>
  auto set_default_filter(Args... args)
      -> std::enable_if_t<std::is_base_of<on_updated_filter_type, Filter>::value> {
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    filter_initializer_ = [args...] { return std::make_shared<Filter>(args...); };
  }

private:
  mutable std::shared_timed_mutex mutex_;

  /// ロボットの生データを取得するFrameのメンバ関数へのポインタ
  /// このポインタを各チームカラーに対して特殊化することで, 同じ更新処理を使えるようにしている
  static const source_function_pointer_type src_;

  /// 最終的な値
  robots_list_type robots_;

  /// 各カメラで検出されたロボットの生データ (KeyはカメラID)
  std::unordered_map<unsigned int, raw_data_array_type> raw_robots_;
  /// 検出された中から選ばれた, 各IDをの最も確かとされる値のリスト
  robots_list_type reliable_robots_;

  /// 更新タイミングがon_updatedなFilter
  std::unordered_map<unsigned int, std::shared_ptr<on_updated_filter_type>> on_updated_filters_;
  /// 更新タイミングがmanualなFilter
  std::unordered_map<unsigned int, std::shared_ptr<manual_filter_type>> manual_filters_;
  /// Filterを初期化するための関数オブジェクト
  std::function<std::shared_ptr<on_updated_filter_type>()> filter_initializer_;
};

} // namespace updater
} // namespace model
} // namespace ai_server

#endif // AI_SERVER_MODEL_UPDATER_ROBOT_H
