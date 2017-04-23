#include <algorithm>
#include <stdexcept>
#include <boost/format.hpp>
#include <boost/variant.hpp>

#include "driver.h"

namespace ai_server {

driver::driver(boost::asio::io_service& io_service,
               std::chrono::high_resolution_clock::duration cycle, const model::world& world)
    : timer_(io_service), cycle_(cycle), world_(world) {}

void driver::register_robot(bool is_yellow, unsigned int id, controller_ptr controller,
                            sender_ptr sender) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto& robots = is_yellow ? robots_yellow_ : robots_blue_;
  robots.emplace(
      id, std::forward_as_tuple(model::command{id}, std::move(controller), std::move(sender)));
}

void driver::unregister_robot(bool is_yellow, unsigned int id) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto& robots = is_yellow ? robots_yellow_ : robots_blue_;
  robots.erase(id);
}

void driver::update_command(bool is_yellow, const model::command& command) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto& robots = is_yellow ? robots_yellow_ : robots_blue_;

  // ロボットが登録されていなかったらエラー
  if (robots.count(command.id()) == 0) {
    throw std::runtime_error(
        boost::str(boost::format("driver: %1% robot id %2% is not registered") %
                   (is_yellow ? "yellow" : "blue") % command.id()));
  }

  std::get<0>(robots.at(command.id())) = command;
}

} // namespace ai_server
