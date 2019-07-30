#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <sstream>
#include <thread>

#include <boost/asio.hpp>
#include <boost/format.hpp>
#include <boost/math/constants/constants.hpp>

#include <gtkmm.h>

#include "ai_server/controller/state_feedback_controller.h"
#include "ai_server/driver.h"
#include "ai_server/filter/state_observer/ball.h"
#include "ai_server/filter/va_calculator.h"
#include "ai_server/game/formation/first_formation.h"
#include "ai_server/model/team_color.h"
#include "ai_server/model/world.h"
#include "ai_server/model/updater/refbox.h"
#include "ai_server/model/updater/world.h"
#include "ai_server/receiver/refbox.h"
#include "ai_server/receiver/vision.h"
#include "ai_server/sender/grsim.h"
#include "ai_server/sender/kiks.h"
#include "ai_server/util/math/affine.h"
#include "ai_server/util/time.h"

using namespace std::chrono_literals;
using namespace std::string_literals;

namespace controller = ai_server::controller;
namespace filter     = ai_server::filter;
namespace game       = ai_server::game;
namespace model      = ai_server::model;
namespace receiver   = ai_server::receiver;
namespace sender     = ai_server::sender;
namespace util       = ai_server::util;

// 60fpsの時にn framesにかかる時間を表現する型
using fps60_type = std::chrono::duration<util::time_point_type::rep, std::ratio<1, 60>>;

// 攻撃方向
enum class dir { right, left };

// 基本設定
// --------------------------------

// WorldModelの設定
static constexpr auto use_va_filter = true; // ロボットの速度/加速度を計算する
static constexpr auto use_ball_observer = true; // ボールの状態オブザーバを有効にする

// Visionの設定
static constexpr char vision_address[] = "224.5.23.2";
static constexpr short vision_port     = 10006;
static constexpr int num_cameras       = 8;

// Refboxの設定
static constexpr char global_refbox_address[] = "224.5.23.1";
static constexpr short global_refbox_port     = 10003;
static constexpr char local_refbox_address[]  = "224.5.23.12";
static constexpr short local_refbox_port      = 10012;

// Senderの設定
static constexpr bool is_grsim            = false;
static constexpr char xbee_path[]         = "/dev/ttyUSB0";
static constexpr char grsim_address[]     = "127.0.0.1";
static constexpr short grsim_command_port = 20011;

// 制御周期の設定
static constexpr auto cycle = std::chrono::duration_cast<util::duration_type>(fps60_type{1});

// stopgame時の速度制限
static constexpr double velocity_limit_at_stopgame = 800.0;

// 簡易loggerの設定
static constexpr auto use_logger = true;

// 簡易logegr
// --------------------------------
namespace logger {
namespace detail {
struct info {
  static constexpr auto symbol = "[*] ";
};
struct success {
  static constexpr auto symbol = "[+] ";
};
struct error {
  static constexpr auto symbol = "[ERROR] ";
};
static std::mutex mutex;
template <class Type, class Stream, class String>
inline void log(Stream&& stream, String&& str) {
  if (!use_logger) return;
  std::lock_guard<std::mutex> lock(mutex);
  stream << Type::symbol << std::forward<String>(str) << std::endl;
}
} // namespace detail
template <class String>
inline void info(String&& str) {
  detail::log<detail::info>(std::cout, std::forward<String>(str));
}
template <class String>
inline void success(String&& str) {
  detail::log<detail::success>(std::cout, std::forward<String>(str));
}
template <class String>
inline void error(String&& str) {
  detail::log<detail::error>(std::cerr, std::forward<String>(str));
}
} // namespace logger

// Gameを行うクラス
// --------------------------------
class game_runner {
public:
  game_runner(model::updater::world& world, model::updater::refbox& refbox1,
              model::updater::refbox& refbox2, std::shared_ptr<sender::base>& sender)
      : running_{false},
        game_thread_{},
        driver_thread_{},
        team_color_(model::team_color::yellow),
        updater_world_(world),
        updater_refbox1_(refbox1),
        updater_refbox2_(refbox2),
        is_global_refbox_{true},
        sender_(sender),
        driver_(driver_io_, cycle, updater_world_, team_color_),
        active_robots_({0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u}) {
    driver_thread_ = std::thread([this] {
      try {
        driver_io_.run();
      } catch (const std::exception& e) {
        logger::error(boost::format("exception at driver_thread\n\t%1%") % e.what());
      } catch (...) {
        logger::error("unknown exception at driver_thread");
      }
    });

    for (auto id : active_robots_) {
      constexpr auto cycle_count = std::chrono::duration<double>(cycle).count();
      auto controller =
          std::make_unique<controller::state_feedback_controller>(cycle_count, world_);
      driver_.register_robot(id, std::move(controller), sender_);
    }
  }

  ~game_runner() {
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    running_ = false;
    if (game_thread_.joinable()) game_thread_.join();
    driver_io_.stop();
    if (driver_thread_.joinable()) driver_thread_.join();
  }

  void start() {
    if (!game_thread_.joinable()) {
      running_     = true;
      game_thread_ = std::thread([this] { main_loop(); });
    }
  }

  void stop() {
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    running_ = false;
    game_thread_.join();
    logger::info("game stopped!");
  }

  std::vector<unsigned int> active_robots() const {
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    return active_robots_;
  }

  void set_active_robots(const std::vector<unsigned int>& ids) {
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    bool changed = false;
    // register new robots
    for (auto id : ids) {
      if (!driver_.registered(id)) {
        constexpr auto cycle_count = std::chrono::duration<double>(cycle).count();
        auto controller =
            std::make_unique<controller::state_feedback_controller>(cycle_count, world_);
        driver_.register_robot(id, std::move(controller), sender_);
        changed = true;
      }
    }
    // unregister robots
    for (auto id : active_robots_) {
      if (std::find(ids.cbegin(), ids.cend(), id) == ids.cend()) {
        driver_.unregister_robot(id);
        changed = true;
      }
    }
    if (changed) {
      std::stringstream ss{};
      ss << "active_robots changed: { ";
      std::copy(active_robots_.cbegin(), active_robots_.cend(),
                std::ostream_iterator<unsigned int>(ss, ", "));
      ss << "} -> { ";
      std::copy(ids.cbegin(), ids.cend(), std::ostream_iterator<unsigned int>(ss, ", "));
      ss << "}";
      logger::info(ss.str());

      active_robots_ = ids;
      reset_formation();
    }
  }

  void set_team_color(model::team_color color) {
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    if (team_color_ != color) {
      driver_.set_team_color(color);
      team_color_ = color;
      reset_formation();
    }
  }

  bool is_global_refbox() const {
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    return is_global_refbox_;
  }

  void use_global_refbox(bool is_global) {
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    if (is_global_refbox_ != is_global) {
      logger::info("switched to "s + (is_global ? "global"s : "local"s) + " refbox");
      is_global_refbox_ = is_global;
    }
  }

  void set_transformation_matrix(double x, double y, double theta) {
    const auto mat = util::math::make_transformation_matrix(x, y, theta);
    updater_world_.set_transformation_matrix(mat);
    updater_refbox1_.set_transformation_matrix(mat);
    updater_refbox2_.set_transformation_matrix(mat);
  }

private:
  void main_loop() {
    logger::success("game started!");
    formation_.reset();

    ai_server::util::time_point_type prev_time{};

    while (running_) {
      try {
        const auto current_time = util::clock_type::now();
        if (current_time - prev_time < cycle) {
          // std::this_thread::yield();
          std::this_thread::sleep_for(1ms);
        } else {
          std::unique_lock<std::shared_timed_mutex> lock(mutex_);

          const auto prev_cmd = refbox_.command();

          world_  = updater_world_.value();
          refbox_ = (is_global_refbox_ ? updater_refbox1_ : updater_refbox2_).value();

          const auto current_cmd = refbox_.command();

          if (current_cmd != prev_cmd) {
            if (current_cmd == model::refbox::game_command::stop) {
              driver_.set_velocity_limit(velocity_limit_at_stopgame);
            } else {
              driver_.set_velocity_limit(std::numeric_limits<double>::max());
            }
          }

          const auto visible_robots =
              static_cast<bool>(team_color_) ? world_.robots_yellow() : world_.robots_blue();

          if (std::all_of(active_robots_.cbegin(), active_robots_.cend(),
                          [&visible_robots](auto&& id) { return visible_robots.count(id); })) {
            evalute_formation();
          }

          prev_time = current_time;
        }
      } catch (const std::exception& e) {
        logger::error(boost::format("exception at game_thread\n\t%1%") % e.what());
      } catch (...) {
        logger::error("unknown exception at game_thread");
      }
    }
  }

  void reset_formation() {
    formation_ = std::make_unique<game::formation::first_formation>(
        world_, refbox_, static_cast<bool>(team_color_), active_robots_);
    logger::info("formation resetted");
  }

  void evalute_formation() {
    if (!formation_) reset_formation();
    auto agents = formation_->execute();
    for (auto agent : agents) {
      auto actions = agent->execute();
      for (auto action : actions) {
        auto command = action->execute();
        driver_.update_command(command);
      }
    }
  }

  mutable std::shared_timed_mutex mutex_;
  std::atomic<bool> running_;
  std::thread game_thread_;
  std::thread driver_thread_;
  model::team_color team_color_;
  model::updater::world& updater_world_;
  model::updater::refbox& updater_refbox1_;
  model::updater::refbox& updater_refbox2_;
  bool is_global_refbox_;

  boost::asio::io_service driver_io_;
  std::shared_ptr<sender::base> sender_;
  ai_server::driver driver_;

  model::world world_;
  model::refbox refbox_;
  std::vector<unsigned int> active_robots_;

  std::unique_ptr<game::formation::base> formation_;
};

// Gameを行うクラス
// --------------------------------
class game_window final : public Gtk::Window {
public:
  game_window(model::updater::world& world, game_runner& runner)
      : updater_world_(world), runner_(runner) {
    set_border_width(10);
    set_default_size(800, 500);
    set_title("Game configurations");

    {
      // left widgets
      color_.set_label("Team color");
      color_r1_.set_label("Yellow");
      color_r2_.set_label("Blue");
      color_r2_.join_group(color_r1_);
      color_box_.set_border_width(4);
      color_box_.pack_start(color_r1_);
      color_box_.pack_start(color_r2_);
      color_.add(color_box_);
      left_.pack_start(color_, Gtk::PACK_SHRINK, 4);

      dir_.set_label("Attack direction");
      dir_r1_.set_label("Right");
      dir_r2_.set_label("Left");
      dir_r2_.join_group(dir_r1_);
      dir_box_.set_border_width(4);
      dir_box_.pack_start(dir_r1_);
      dir_box_.pack_start(dir_r2_);
      dir_.add(dir_box_);
      left_.pack_start(dir_, Gtk::PACK_SHRINK, 4);

      robots_.set_label("Active robots");
      robots_id_box_.set_border_width(4);
      robots_id_box_.set_max_children_per_line(6);
      for (auto i = 0u; i < 12; ++i) {
        robots_cb_.emplace_back(std::to_string(i));
        robots_id_box_.add(robots_cb_.back());
      }
      apply_.set_label("Apply");
      apply_.set_border_width(4);
      apply_.set_sensitive(false);
      apply_.signal_clicked().connect(
          sigc::mem_fun(*this, &game_window::handle_change_active_robots));
      robots_box_.pack_start(robots_id_box_);
      robots_box_.pack_start(apply_);
      robots_.add(robots_box_);
      left_.pack_start(robots_, Gtk::PACK_SHRINK, 4);

      camera_.set_label("Camera");
      camera_id_box_.set_border_width(4);
      camera_id_box_.set_max_children_per_line(6);
      for (auto i = 0u; i < num_cameras; ++i) {
        camera_cb_.emplace_back(std::to_string(i));
        camera_id_box_.add(camera_cb_.back());
      }
      camera_.add(camera_id_box_);
      left_.pack_start(camera_, Gtk::PACK_SHRINK, 4);

      refbox_.set_label("Refbox");
      refbox_r1_.set_label("Global");
      refbox_r2_.set_label("Local");
      refbox_r2_.join_group(refbox_r1_);
      refbox_box_.set_border_width(4);
      refbox_box_.pack_start(refbox_r1_);
      refbox_box_.pack_start(refbox_r2_);
      refbox_.add(refbox_box_);
      left_.pack_start(refbox_, Gtk::PACK_SHRINK, 4);

      start_.set_label("start");
      start_.set_sensitive(false);
      start_.signal_clicked().connect(sigc::mem_fun(*this, &game_window::handle_start_stop));
      left_.pack_end(start_, Gtk::PACK_SHRINK);

      left_.set_size_request(320, -1);
      center_.pack_start(left_, Gtk::PACK_SHRINK, 10);
    }

    {
      // right widgets
      init_tree();
      center_.pack_end(tree_, Gtk::PACK_EXPAND_WIDGET, 10);
    }

    add(center_);
    show_all_children();

    init_radio_buttons();
    init_check_buttons();
  }

  void set_ready() {
    start_.set_sensitive(true);
  }

private:
  void init_tree() {
    treestore_ = Gtk::TreeStore::create(model_);
    tree_.set_model(treestore_);
    tree_.append_column_editable("Agent/Action", model_.name_);
    tree_.append_column("Robot ID", model_.id_);

    auto r1            = *(treestore_->append());
    r1[model_.name_]   = "stopgame";
    auto r1_1          = *(treestore_->append(r1.children()));
    r1_1[model_.name_] = "no_op";
    r1_1[model_.id_]   = 0;
    auto r1_2          = *(treestore_->append(r1.children()));
    r1_2[model_.name_] = "no_op";
    r1_2[model_.id_]   = 1;
    auto r1_3          = *(treestore_->append(r1.children()));
    r1_3[model_.name_] = "no_op";
    r1_3[model_.id_]   = 2;

    tree_.expand_all();
  }

  void init_radio_buttons() {
    dir_r1_.signal_toggled().connect([this] {
      runner_.set_transformation_matrix(
          0.0, 0.0, dir_r1_.get_active() ? 0.0 : boost::math::double_constants::pi);
    });

    color_r1_.signal_toggled().connect([this] {
      if (color_r1_.get_active()) {
        runner_.set_team_color(model::team_color::yellow);
      } else {
        runner_.set_team_color(model::team_color::blue);
      }
    });

    if (runner_.is_global_refbox()) {
      refbox_r1_.set_active(true);
    } else {
      refbox_r2_.set_active(true);
    }
    refbox_r1_.signal_toggled().connect([this] {
      if (refbox_r1_.get_active()) {
        runner_.use_global_refbox(true);
      } else {
        runner_.use_global_refbox(false);
      }
    });
  }

  void init_check_buttons() {
    auto robots = runner_.active_robots();
    for (auto i = 0u; i < 12; ++i) {
      auto& cb = robots_cb_.at(i);
      cb.set_active(std::find(robots.cbegin(), robots.cend(), i) != robots.cend());
      cb.signal_toggled().connect(
          sigc::mem_fun(*this, &game_window::handle_active_robots_changed));
    }

    for (auto i = 0u; i < num_cameras; ++i) {
      auto& cb = camera_cb_.at(i);
      cb.set_active(updater_world_.is_camera_enabled(i));
      cb.signal_toggled().connect([id = i, this] {
        auto& cb = camera_cb_.at(id);
        if (cb.get_active()) {
          updater_world_.enable_camera(id);
          logger::info(boost::format("camera%1% enabled") % id);
        } else {
          updater_world_.disable_camera(id);
          logger::info(boost::format("camera%1% disabled") % id);
        }
      });
    }
  }

  void handle_start_stop() {
    if (start_.get_label() == "start") {
      runner_.start();
      start_.set_label("stop");
    } else {
      runner_.stop();
      start_.set_label("start");
    }
  }

  void handle_active_robots_changed() {
    apply_.set_sensitive(true);
  }

  void handle_change_active_robots() {
    std::vector<unsigned int> robots{};
    for (auto i = 0u; i < robots_cb_.size(); ++i) {
      if (robots_cb_[i].get_active()) {
        robots.push_back(i);
      }
    }
    runner_.set_active_robots(robots);
    apply_.set_sensitive(false);
  }

  class tree_model : public Gtk::TreeModel::ColumnRecord {
  public:
    tree_model() {
      add(name_);
      add(id_);
    }

    Gtk::TreeModelColumn<Glib::ustring> name_;
    Gtk::TreeModelColumn<unsigned int> id_;
  };

  tree_model model_;

  Gtk::Frame color_, dir_, robots_, refbox_, camera_;
  Gtk::VBox left_, robots_box_;
  Gtk::HBox center_, color_box_, dir_box_, refbox_box_;
  Gtk::RadioButton color_r1_, color_r2_, dir_r1_, dir_r2_, refbox_r1_, refbox_r2_;
  std::vector<Gtk::CheckButton> robots_cb_, camera_cb_;
  Gtk::FlowBox robots_id_box_, camera_id_box_;
  Gtk::Button apply_, start_;
  Gtk::TreeView tree_;
  Glib::RefPtr<Gtk::TreeStore> treestore_;

  model::updater::world& updater_world_;
  game_runner& runner_;
};

auto main(int argc, char** argv) -> int {
  logger::success("(⋈◍＞◡＜◍)。✧♡");

  boost::asio::io_service receiver_io{};

  // Receiver, Driverなどを回すスレッド
  std::thread io_thread{};

  try {
    // WorldModelの設定
    model::updater::world updater_world{};
    {
      // ロボットの速度と加速度を計算するか
      if (use_va_filter) {
        updater_world.robots_blue_updater()
            .set_default_filter<filter::va_calculator<model::robot>>();
        updater_world.robots_yellow_updater()
            .set_default_filter<filter::va_calculator<model::robot>>();
      }
      logger::info("va filter (robot): "s + (use_va_filter ? "enabled"s : "disabled"s));
      // ボールの状態オブザーバを使うか
      if (use_ball_observer) {
        updater_world.ball_updater().set_filter<filter::state_observer::ball>(
            model::ball{}, util::clock_type::now());
      }
      logger::info("state observer (ball): "s + (use_ball_observer ? "enabled"s : "disabled"s));
    }

    // Vision receiverの設定
    std::atomic<bool> vision_received{false};
    receiver::vision vision{receiver_io, "0.0.0.0", vision_address, vision_port};
    vision.on_receive([&updater_world, &vision_received](auto&& p) {
      if (!vision_received) {
        // 最初に受信したときにメッセージを表示する
        logger::success("vision packet received!");
        vision_received = true;
      }
      updater_world.update(std::forward<decltype(p)>(p));
    });
    logger::info(boost::format("vision: %1%:%2%") % vision_address % vision_port);

    // Refbox receiverの設定
    std::atomic<bool> refbox1_received{false};
    model::updater::refbox updater_refbox1_{};
    receiver::refbox refbox1{receiver_io, "0.0.0.0", global_refbox_address, global_refbox_port};
    refbox1.on_receive([&updater_refbox1_, &refbox1_received](auto&& p) {
      if (!refbox1_received) {
        // 最初に受信したときにメッセージを表示する
        logger::success("refbox (global) packet received!");
        refbox1_received = true;
      }

      updater_refbox1_.update(std::forward<decltype(p)>(p));
    });
    logger::info(boost::format("global refbox: %1%:%2%") % global_refbox_address %
                 global_refbox_port);

    std::atomic<bool> refbox2_received{false};
    model::updater::refbox updater_refbox2_{};
    receiver::refbox refbox2{receiver_io, "0.0.0.0", local_refbox_address, local_refbox_port};
    refbox2.on_receive([&updater_refbox2_, &refbox2_received](auto&& p) {
      if (!refbox2_received) {
        // 最初に受信したときにメッセージを表示する
        logger::success("refbox (local) packet received!");
        refbox2_received = true;
      }

      updater_refbox2_.update(std::forward<decltype(p)>(p));
    });
    logger::info(boost::format("local refbox: %1%:%2%") % local_refbox_address %
                 local_refbox_port);

    // receiver_ioに登録されたタスクを別スレッドで開始
    io_thread = std::thread{[&receiver_io] {
      try {
        receiver_io.run();
      } catch (std::exception& e) {
        logger::error(boost::format("exception at io_thread: %1%") % e.what());
      }
    }};

    // Senderの設定
    std::shared_ptr<sender::base> sender{};
    if (is_grsim) {
      sender = std::make_shared<sender::grsim>(receiver_io, grsim_address, grsim_command_port);
      logger::info(boost::format("sender: grSim (%1%:%2%)") % grsim_address %
                   grsim_command_port);
    } else {
      sender = std::make_shared<sender::kiks>(receiver_io, xbee_path);
      logger::info(boost::format("sender: kiks (%1%)") % xbee_path);
    }

    auto app = Gtk::Application::create(argc, argv, "org.gtkmm.example");

    game_runner runner{updater_world, updater_refbox1_, updater_refbox2_, sender};
    game_window gw{updater_world, runner};
    gw.show();

    std::thread wait([&runner, &gw, &vision_received] {
      // Visionから値を受信するまで待つ
      do {
        std::this_thread::sleep_for(500ms);
      } while (!vision_received);
      // 状態オブザーバなどの値が収束するまで待つ
      std::this_thread::sleep_for(5s);
      logger::success("ready!");
      gw.set_ready();
    });

    app->run(gw);

    wait.detach();
    receiver_io.stop();
    io_thread.join();
  } catch (std::exception& e) {
    logger::error(e.what());
    receiver_io.stop();
    io_thread.join();
    std::quick_exit(-1);
  } catch (...) {
    logger::error("unknown error occurred");
    receiver_io.stop();
    io_thread.join();
    std::quick_exit(-1);
  }
}
