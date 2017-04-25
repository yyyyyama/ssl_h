#include <algorithm>
#include <stdexcept>
#include <boost/format.hpp>
#include <boost/variant.hpp>

#include "driver.h"

namespace ai_server {

driver::driver(boost::asio::io_service& io_service,
               std::chrono::high_resolution_clock::duration cycle, const model::world& world)
    : timer_(io_service), cycle_(cycle), world_(world) {
  // タイマが開始されたらdriver::main_loop()が呼び出されるように設定
  timer_.async_wait([this](auto&& error) { main_loop(std::forward<decltype(error)>(error)); });
}

void driver::register_robot(bool is_yellow, unsigned int id, controller_ptr controller,
                            sender_ptr sender) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto& robot_params = is_yellow ? robots_yellow_params_ : robots_blue_params_;
  robot_params.emplace(
      id, std::forward_as_tuple(model::command{id}, std::move(controller), std::move(sender)));
}

void driver::unregister_robot(bool is_yellow, unsigned int id) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto& robot_params = is_yellow ? robots_yellow_params_ : robots_blue_params_;
  robot_params.erase(id);
}

void driver::update_command(bool is_yellow, const model::command& command) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto& robot_params = is_yellow ? robots_yellow_params_ : robots_blue_params_;

  // ロボットが登録されていなかったらエラー
  if (robot_params.count(command.id()) == 0) {
    throw std::runtime_error(
        boost::str(boost::format("driver: %1% robot id %2% is not registered") %
                   (is_yellow ? "yellow" : "blue") % command.id()));
  }

  std::get<0>(robot_params.at(command.id())) = command;
}

void driver::main_loop(const boost::system::error_code& error) {
  // TODO: エラーが発生したことを上の階層に伝える仕組みを実装する
  if (error) return;

  // 処理の開始時刻を記録
  const auto start_time = std::chrono::high_resolution_clock::now();

  std::lock_guard<std::mutex> lock(mutex_);

  // 登録されたロボットの命令をControllerを通してから送信する
  for (auto&& rp : robots_blue_params_) process(false, rp.second);
  for (auto&& rp : robots_yellow_params_) process(true, rp.second);

  // 処理の開始時刻からcycle_経過した後に再度main_loop()が呼び出されるように設定
  timer_.expires_at(start_time + cycle_);
  timer_.async_wait([this](auto&& error) { main_loop(std::forward<decltype(error)>(error)); });
}

void driver::process(bool is_yellow, driver_param_type& driver_param) {
  auto command      = std::get<0>(driver_param);
  const auto id     = command.id();
  const auto robots = is_yellow ? world_.robots_yellow() : world_.robots_blue();

  // ロボットが検出されていないときは何もしない
  if (robots.count(id) == 0) return;

  // commandの指令値をControllerに通す
  auto controller = [ id, &c = std::get<1>(driver_param), &r = robots.at(id) ](auto&& s) {
    return c->operator()(r, std::forward<decltype(s)>(s));
  };
  command.set_velocity(boost::apply_visitor(controller, command.setpoint()));

  // Senderで送信
  std::get<2>(driver_param)->send_command(command);
}

} // namespace ai_server
