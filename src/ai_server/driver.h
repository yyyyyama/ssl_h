#ifndef AI_SERVER_DRIVER_H
#define AI_SERVER_DRIVER_H

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <tuple>
#include <boost/asio.hpp>
#include <boost/asio/basic_waitable_timer.hpp>
#include <boost/signals2.hpp>

#include "ai_server/controller/base.h"
#include "ai_server/model/command.h"
#include "ai_server/model/team_color.h"
#include "ai_server/model/updater/world.h"
#include "ai_server/sender/base.h"
#include "ai_server/util/time.h"

namespace ai_server {

class driver {
  /// Controllerのポインタの型
  using controller_type = std::unique_ptr<controller::base>;
  /// Senderのポインタの型
  using sender_type = std::shared_ptr<sender::base>;
  /// Driverで行う処理で必要となる各ロボットの情報の型
  using metadata_type = std::tuple<model::command, controller_type, sender_type>;
  /// Commandが更新された(Controllerを通された)ときに発火するシグナルの型
  using updated_signal_type = boost::signals2::signal<void(const model::command&)>;

public:
  /// @param cycle            制御周期
  /// @param world            updater::worldの参照
  /// @param color            チームカラー
  driver(boost::asio::io_service& io_service, util::duration_type cycle,
         const model::updater::world& world, model::team_color color);

  /// @brief                  現在設定されているチームカラーを取得する
  model::team_color team_color() const;

  /// @brief                  チームカラーを変更する
  /// @param color            チームカラー
  void set_team_color(model::team_color color);

  /// @brief                  Driverにロボットを登録する
  /// @param id               ロボットのID
  /// @param controller       Controller
  /// @param sender           Sender
  void register_robot(unsigned int id, controller_type controller, sender_type sender);

  /// @brief                  Driverに登録されたロボットを解除する
  /// @param id               ロボットのID
  void unregister_robot(unsigned int id);

  /// @brief                  ロボットがDriverに登録されているか調べる
  /// @param id               ロボットのID
  bool registered(unsigned int id) const;

  /// @brief                  ロボットへの命令を更新する
  /// @param command          ロボットへの命令
  void update_command(const model::command& command);

  /// @brief                  commandが更新されたときに呼ばれる関数を登録する
  /// @param slot             commandが更新されたときに呼びたい関数
  boost::signals2::connection on_command_updated(const updated_signal_type::slot_type& slot);

  /// @brief                  登録されているロボットのControllerに速度制限をかける
  /// @param limit            速度の制限値
  void set_velocity_limit(double limit);

private:
  /// @brief                  cycle_毎に呼ばれる制御部のメインループ
  void main_loop(const boost::system::error_code& error);

  /// @brief                  ロボットへの命令をControllerを通してから送信する
  void process(const model::world& world, metadata_type& metadata);

  mutable std::mutex mutex_;

  /// 制御部の処理を一定の周期で回すためのタイマ
  boost::asio::basic_waitable_timer<util::clock_type> timer_;
  /// 制御周期
  util::duration_type cycle_;

  /// updater::worldの参照
  const model::updater::world& world_;

  /// チームカラー
  model::team_color team_color_;

  /// 登録されたロボットの情報
  std::unordered_map<unsigned int, metadata_type> robots_metadata_;

  updated_signal_type command_updated_;
};

} // namespace ai_server

#endif // AI_SERVER_DRIVER_H
