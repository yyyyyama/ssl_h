#include <algorithm>
#include <stdexcept>
#include <variant>
#include <boost/format.hpp>

#include "driver.h"

namespace ai_server {

driver::driver(boost::asio::io_context& io_context, util::duration_type cycle,
               const model::updater::world& world, model::team_color color)
    : timer_(io_context), cycle_(cycle), world_(world), team_color_(color) {
  // タイマが開始されたらdriver::main_loop()が呼び出されるように設定
  timer_.async_wait([this](auto&& error) { main_loop(std::forward<decltype(error)>(error)); });
}

model::team_color driver::team_color() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return team_color_;
}

void driver::set_team_color(model::team_color color) {
  std::lock_guard<std::mutex> lock(mutex_);
  team_color_ = color;
}

boost::signals2::connection driver::on_command_updated(
    const updated_signal_type::slot_type& slot) {
  return command_updated_.connect(slot);
}

void driver::set_velocity_limit(double limit) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto&& meta : robots_metadata_) std::get<1>(meta.second)->set_velocity_limit(limit);
}

void driver::set_stable(const bool stable) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto&& meta : robots_metadata_) std::get<1>(meta.second)->set_stable(stable);
}

void driver::register_robot(unsigned int id, controller_type controller, sender_type sender) {
  std::lock_guard<std::mutex> lock(mutex_);
  robots_metadata_.emplace(
      id, std::forward_as_tuple(model::command{id}, std::move(controller), std::move(sender)));
}

void driver::unregister_robot(unsigned int id) {
  std::lock_guard<std::mutex> lock(mutex_);
  robots_metadata_.erase(id);
}

bool driver::registered(unsigned int id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return robots_metadata_.count(id);
}

void driver::update_command(const model::command& command) {
  std::lock_guard<std::mutex> lock(mutex_);

  // ロボットが登録されていなかったらエラー
  const auto id = command.id();
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
  const auto start_time = util::clock_type::now();

  std::lock_guard<std::mutex> lock(mutex_);

  // このループでのWorldModelを生成
  const auto world = world_.value();

  // 登録されたロボットの命令をControllerを通してから送信する
  for (auto&& meta : robots_metadata_) process(world, meta.second);

  // 処理の開始時刻からcycle_経過した後に再度main_loop()が呼び出されるように設定
  timer_.expires_at(start_time + cycle_);
  timer_.async_wait([this](auto&& error) { main_loop(std::forward<decltype(error)>(error)); });
}

void driver::process(const model::world& world, metadata_type& metadata) {
  auto& [command, controller, sender] = metadata;

  const auto id = command.id();
  const auto robots =
      static_cast<bool>(team_color_) ? world.robots_yellow() : world.robots_blue();

  const auto field = world.field();

  // ロボットが検出されていないときは何もしない
  if (const auto it = robots.find(id); it != robots.cend()) {
    const auto& robot = it->second;

    // 指令値を Controller に通して速度を得る
    auto c = [&robot, &field, &c = *controller](auto&& s) {
      return c.update(robot, field, std::forward<decltype(s)>(s));
    };
    const auto velocity = std::visit(c, command.setpoint());

    // 実際に送信する命令
    auto actual_command = command;
    actual_command.set_velocity(velocity);

    // Senderで送信
    sender->send_command(actual_command, team_color_);

    // 登録された関数があればそれを呼び出す
    command_updated_(actual_command);
  }
}

} // namespace ai_server
