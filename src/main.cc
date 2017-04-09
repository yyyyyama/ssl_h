#include <algorithm>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <thread>
#include <boost/asio.hpp>
#include <boost/format.hpp>

#include "ai_server/controller/pid_controller.h"
#include "ai_server/game/action/marking.h"
#include "ai_server/model/command.h"
#include "ai_server/model/world.h"
#include "ai_server/receiver/vision.h"
#include "ai_server/sender/kiks.h"
#include "ai_server/sender/grsim.h"

using namespace std::chrono_literals;
using namespace std::string_literals;
using namespace ai_server;

// --------------------------------

// ロボットの設定
static constexpr auto id        = 3u;
static constexpr bool is_yellow = true;

// Vision の設定
static constexpr char vision_address[] = "224.5.23.2";
static constexpr short vision_port     = 10020;

// Sender の設定
static constexpr bool is_grsim            = true;
static constexpr char xbee_path[]         = "/dev/ttyUSB0";
static constexpr char grsim_address[]     = "192.168.24.60";
static constexpr short grsim_command_port = 20011;

// 制御周期の設定
static constexpr auto interval = 1.0s / 60;

// --------------------------------

auto main() -> int {
  boost::asio::io_service io_service{};

  // WorldModelとVision receiverの初期化
  model::world w{};
  receiver::vision v{io_service, "0.0.0.0", vision_address, vision_port};
  v.on_receive([&w](auto&& p) {
    // Vision から新しいデータが来た時に WorldModel を更新する
    w.update(std::forward<decltype(p)>(p));
  });

  // Senderの初期化
  std::unique_ptr<sender::base> sender{};
  if (is_grsim) {
    sender = std::make_unique<sender::grsim>(io_service, grsim_address, grsim_command_port);
  } else {
    sender = std::make_unique<sender::kiks>(io_service, xbee_path);
  }

  // Senderの初期化
  controller::pid_controller controller{interval.count()};

  // Actionの初期化
  game::action::marking action{w, is_yellow, id};
  action.mark_robot(3);
	action.mark_mode(0);

  std::thread vth([&io_service] { io_service.run(); });

  for (;;) {
    const auto& robots = is_yellow ? w.robots_yellow() : w.robots_blue();
    // ロボットが見えているか
    if (robots.count(id)) {
      // このループでの命令を計算
      auto cmd = action.execute();

      // setpointをControllerに通す
      auto c = [ id = id, &controller, &robots ](auto&& sp) {
        return controller(robots.at(id), std::forward<decltype(sp)>(sp));
      };
      const auto sp = cmd.setpoint();
      const auto v  = boost::apply_visitor(c, sp);

      // 送信
      cmd.set_velocity(v);
      sender->send_command(cmd);
    }

    std::this_thread::sleep_for(interval);
  }

  vth.join();
}
