#ifndef AI_SERVER_DRIVER_H
#define AI_SERVER_DRIVER_H

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <tuple>
#include <boost/asio.hpp>
#include <boost/asio/high_resolution_timer.hpp>

#include "ai_server/controller/base.h"
#include "ai_server/model/command.h"
#include "ai_server/model/world.h"
#include "ai_server/sender/base.h"

namespace ai_server {

class driver {
  using controller_ptr = std::unique_ptr<controller::base>;
  using sender_ptr     = std::unique_ptr<sender::base>;
  using value_type     = std::tuple<model::command, controller_ptr, sender_ptr>;

  mutable std::mutex mutex_;

  boost::asio::high_resolution_timer timer_;

  /// 制御周期
  std::chrono::high_resolution_clock::duration cycle_;

  /// WorldModelの参照
  const model::world& world_;

  /// 登録された青ロボットの情報
  std::unordered_map<unsigned int, value_type> robots_blue_;
  /// 登録された黄ロボットの情報
  std::unordered_map<unsigned int, value_type> robots_yellow_;

public:
  /// @param cycle            制御周期
  /// @param world            WorldModelの参照
  driver(boost::asio::io_service& io_service,
         std::chrono::high_resolution_clock::duration cycle, const model::world& world);

  /// @brief                  Driverにロボットを登録する
  /// @param is_yellow        ロボットの色
  /// @param id               ロボットのID
  /// @param controller       Controller
  /// @param sender           Sender
  void register_robot(bool is_yellow, unsigned int id, controller_ptr controller,
                      sender_ptr sender);

  /// @brief                  Driverに登録されたロボットを解除する
  /// @param is_yellow        ロボットの色
  /// @param id               ロボットのID
  void unregister_robot(bool is_yellow, unsigned int id);

  /// @brief                  ロボットへの命令を更新する
  /// @param is_yellow        ロボットの色
  /// @param command          ロボットへの命令
  void update_command(bool is_yellow, const model::command& command);

private:
  /// @brief                  cycle_毎に呼ばれる制御部のメインループ
  void main_loop(const boost::system::error_code& error);

  /// @brief                  valueに登録されたcommandをControllerを通してから送信する
  void process_command(bool is_yellow, value_type& value);
};

} // namespace ai_server

#endif // AI_SERVER_DRIVER_H
