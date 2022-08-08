#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <type_traits>
#include <variant>
#include <boost/format.hpp>

#include "ai_server/model/motion/stop.h"
#include "ai_server/model/motion/turn_left.h"
#include "ai_server/model/motion/turn_right.h"
#include "ai_server/model/motion/walk_backward.h"
#include "ai_server/model/motion/walk_forward.h"
#include "ai_server/model/motion/walk_left.h"
#include "ai_server/model/motion/walk_right.h"
#include "ai_server/model/motion/getup.h"
#include "driver.h"
#include <iostream>  //omega表示のため
using namespace std; // omega表示のため

namespace ai_server {

driver::driver(boost::asio::io_context& io_context, std::chrono::steady_clock::duration cycle,
               const model::updater::world& world, model::team_color color)
    : timer_(io_context), cycle_(cycle), world_(world), team_color_(color) {
  // タイマが開始されたらdriver::main_loop()が呼び出されるように設定
  timer_.async_wait([this](auto&& error) { main_loop(std::forward<decltype(error)>(error)); });
}

model::team_color driver::team_color() const {
  std::unique_lock lock(mutex_);
  return team_color_;
}

void driver::set_team_color(model::team_color color) {
  std::unique_lock lock(mutex_);
  team_color_ = color;
}

boost::signals2::connection driver::on_command_updated(
    const updated_signal_type::slot_type& slot) {
  return command_updated_.connect(slot);
}

boost::signals2::connection driver::on_command_updated_extended(
    const updated_signal_type::extended_slot_type& slot) {
  return command_updated_.connect_extended(slot);
}

void driver::set_velocity_limit(double limit) {
  std::unique_lock lock(mutex_);
  for (auto&& meta : robots_metadata_) std::get<1>(meta.second)->set_velocity_limit(limit);
}

void driver::set_stable(const bool stable) {
  std::unique_lock lock(mutex_);
  for (auto&& meta : robots_metadata_) std::get<1>(meta.second)->set_stable(stable);
}

void driver::register_robot(unsigned int id, controller_type controller, radio_type radio) {
  std::unique_lock lock(mutex_);
  robots_metadata_.emplace(
      std::piecewise_construct, std::forward_as_tuple(id),
      std::forward_as_tuple(model::command{}, std::move(controller), std::move(radio)));
}

void driver::unregister_robot(unsigned int id) {
  std::unique_lock lock(mutex_);
  robots_metadata_.erase(id);
}

bool driver::registered(unsigned int id) const {
  std::unique_lock lock(mutex_);
  return robots_metadata_.count(id);
}

void driver::update_command(unsigned int id, const model::command& command) {
  std::unique_lock lock(mutex_);

  // ロボットが登録されていなかったらエラー
  if (auto it = robots_metadata_.find(id); it != robots_metadata_.end()) {
    std::get<0>(it->second) = command;
  } else {
    throw std::runtime_error(
        boost::str(boost::format("driver: %1% robot id %2% is not registered") %
                   (static_cast<bool>(team_color_) ? "yellow" : "blue") % id));
  }
}

void driver::main_loop(const boost::system::error_code& error) {
  // TODO: エラーが発生したことを上の階層に伝える仕組みを実装する
  if (error) return;

  // 処理の開始時刻を記録
  const auto start_time = std::chrono::steady_clock::now();

  std::unique_lock lock(mutex_);

  // このループでのWorldModelを生成
  const auto world = world_.value();

  // 登録されたロボットの命令をControllerを通してから送信する
  for (auto&& [id, meta] : robots_metadata_) process(id, meta, world);

  // 処理の開始時刻からcycle_経過した後に再度main_loop()が呼び出されるように設定
  timer_.expires_at(start_time + cycle_);
  timer_.async_wait([this](auto&& error) { main_loop(std::forward<decltype(error)>(error)); });
}

void driver::process(unsigned int id, metadata_type& metadata, const model::world& world) {
  auto& [command, controller, radio] = metadata;

  const auto robots =
      static_cast<bool>(team_color_) ? world.robots_yellow() : world.robots_blue();

  const auto field = world.field();

  // ロボットが検出されていないときは何もしない
  if (const auto it = robots.find(id); it != robots.cend()) {
    const auto& robot = it->second;

        //auto getup = 0;  // mw getup

    // 指令値を Controller に通して速度を得る
    auto c = [&robot, &field, &c = *controller](auto&&... args) {
      return c.update(robot, field, std::forward<decltype(args)>(args)...);
    };
    auto [sp, sp_rot] = command.setpoint_pair();
    if (command.motion()) {
      const auto [mvx, mvy, momega] = command.motion()->execute();
      const auto st                 = std::sin(robot.theta());
      const auto ct                 = std::cos(robot.theta());
      const auto vxf                = ct * mvx - st * mvy;
      const auto vyf                = st * mvx + ct * mvy;
      command.set_velocity(vxf, vyf, momega);
    }
    auto [vx, vy, omega] = std::visit(c, sp, sp_rot);
    if (!command.motion()) {
      // 回転
      constexpr double rot_th = 0.5;

   // std::cout << "omega " <<  omega << "\n";

      if (rot_th < omega) {
        command.set_motion(std::make_shared<model::motion::turn_left>());
      } else if (omega < -rot_th) {
        command.set_motion(std::make_shared<model::motion::turn_right>());
      } else {
        command.set_motion(std::make_shared<model::motion::stop>());
      }

      // 移動
      constexpr double move_th = 100.0;
      if (std::abs(vy) < std::abs(vx)) {
        if (move_th < vx) {
          command.set_motion(std::make_shared<model::motion::walk_forward>());
        } else if (vx < -move_th) {
          command.set_motion(std::make_shared<model::motion::walk_backward>());
        }
      } else {
        if (move_th < vy) {
          command.set_motion(std::make_shared<model::motion::walk_left>());
        } else if (vy < -move_th) {
          command.set_motion(std::make_shared<model::motion::walk_right>());
        }
      }

      const auto [mvx, mvy, momega] = command.motion()->execute();
      vx                            = mvx;
      vy                            = mvy;
      omega                         = momega;
    }

    // 命令の送信
    auto r = std::dynamic_pointer_cast<radio::base::simulator>(radio);
    if (r) {
      radio->send(team_color_, id, command.kick_flag(), command.dribble(), vx, vy, omega);
    } else {
      radio->send(team_color_, id, command.motion());
    }

    // 登録された関数があればそれを呼び出す
    // controller はロボット基準の速度を返すのでフィールド基準にもどす
    const auto st  = std::sin(robot.theta());
    const auto ct  = std::cos(robot.theta());
    const auto vxf = ct * vx - st * vy;
    const auto vyf = st * vx + ct * vy;
    command_updated_(team_color_, id, command.kick_flag(), command.dribble(), vxf, vyf, omega);
  }else{  // mw getup
   //for(int i=0 ; i < 5 ;++i)
          command.set_motion(std::make_shared<model::motion::getup>());}
     }  // mw
//}
} // namespace ai_server
