#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <sstream>
#include <thread>

#include <boost/asio.hpp>
#include <boost/math/constants/constants.hpp>

#include <fmt/chrono.h>
#include <fmt/format.h>
#include <fmt/ranges.h>

#include <gtkmm.h>

#include "ai_server/controller/state_feedback.h"
#include "ai_server/driver.h"
#include "ai_server/filter/state_observer/ball.h"
#include "ai_server/filter/va_calculator.h"
#include "ai_server/game/context.h"
#include "ai_server/game/formation/first_formation.h"
#include "ai_server/game/nnabla.h"
#include "ai_server/logger/logger.h"
#include "ai_server/logger/sink/ostream.h"
#include "ai_server/model/team_color.h"
#include "ai_server/model/world.h"
#include "ai_server/model/updater/refbox.h"
#include "ai_server/model/updater/world.h"
#include "ai_server/radio/connection/serial.h"
#include "ai_server/radio/connection/udp.h"
#include "ai_server/radio/grsim.h"
#include "ai_server/radio/kiks.h"
#include "ai_server/receiver/refbox.h"
#include "ai_server/receiver/vision.h"
#include "ai_server/util/math/affine.h"
#include "ai_server/util/thread.h"
#include "ai_server/util/time.h"

using namespace std::chrono_literals;
using namespace std::string_literals;

namespace controller = ai_server::controller;
namespace filter     = ai_server::filter;
namespace game       = ai_server::game;
namespace logger     = ai_server::logger;
namespace model      = ai_server::model;
namespace radio      = ai_server::radio;
namespace receiver   = ai_server::receiver;
namespace util       = ai_server::util;

// 60fpsの時にn framesにかかる時間を表現する型
using fps60_type =
    std::chrono::duration<std::chrono::steady_clock::time_point::rep, std::ratio<1, 60>>;

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

// Radioの設定
static constexpr bool is_grsim            = false;
static constexpr char xbee_path[]         = "/dev/ttyUSB0";
static constexpr char grsim_address[]     = "127.0.0.1";
static constexpr short grsim_command_port = 20011;

// 制御周期の設定
static constexpr auto cycle =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(fps60_type{1});

// stopgame時の速度制限
static constexpr double velocity_limit_at_stopgame = 1400.0;

// Gameを行うクラス
// --------------------------------
class game_runner {
public:
  game_runner(model::updater::world& world, model::updater::refbox& refbox1,
              model::updater::refbox& refbox2, ai_server::driver& driver,
              std::shared_ptr<radio::base::command> radio)
      : running_{false},
        is_global_refbox_{true},
        need_reset_{false},
        team_color_{model::team_color::yellow},
        updater_world_{world},
        updater_refbox1_{refbox1},
        updater_refbox2_{refbox2},
        active_robots_{0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u},
        driver_{driver},
        radio_{radio} {
    driver_.set_team_color(team_color_);

    for (auto id : active_robots_) {
      constexpr auto cycle_count = std::chrono::duration<double>(cycle).count();
      auto controller            = std::make_unique<controller::state_feedback>(cycle_count);
      driver_.register_robot(id, std::move(controller), radio_);
    }
  }

  ~game_runner() {
    running_ = false;
    cv_.notify_all();
    if (game_thread_.joinable()) game_thread_.join();
  }

  void start() {
    std::unique_lock lock{mutex_};
    if (!game_thread_.joinable()) {
      running_     = true;
      game_thread_ = std::thread([this] { main_loop(); });
      util::set_thread_name(game_thread_, "game_thread");
    }
  }

  void stop() {
    running_ = false;
    cv_.notify_all();
    game_thread_.join();
  }

  std::vector<unsigned int> active_robots() const {
    std::unique_lock lock{mutex_};
    return active_robots_;
  }

  void set_active_robots(const std::vector<unsigned int>& ids) {
    std::unique_lock lock{mutex_};
    bool changed = false;
    // register new robots
    for (auto id : ids) {
      if (!driver_.registered(id)) {
        constexpr auto cycle_count = std::chrono::duration<double>(cycle).count();
        auto controller            = std::make_unique<controller::state_feedback>(cycle_count);
        driver_.register_robot(id, std::move(controller), radio_);
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
      l_.info(fmt::format("active_robots changed: {} -> {}", active_robots_, ids));

      active_robots_ = ids;
      need_reset_    = true;
    }
  }

  void set_team_color(model::team_color color) {
    std::unique_lock lock{mutex_};
    if (team_color_ != color) {
      driver_.set_team_color(color);
      team_color_ = color;
      need_reset_ = true;
    }
  }

  bool is_global_refbox() const {
    std::unique_lock lock{mutex_};
    return is_global_refbox_;
  }

  void use_global_refbox(bool is_global) {
    std::unique_lock lock{mutex_};
    if (is_global_refbox_ != is_global) {
      l_.info("switched to "s + (is_global ? "global"s : "local"s) + " refbox");
      is_global_refbox_ = is_global;
    }
  }

  void set_transformation_matrix(double x, double y, double theta) {
    std::unique_lock lock{mutex_};
    const auto mat = util::math::make_transformation_matrix(x, y, theta);
    updater_world_.set_transformation_matrix(mat);
    updater_refbox1_.set_transformation_matrix(mat);
    updater_refbox2_.set_transformation_matrix(mat);
  }

private:
  void main_loop() {
    l_.info("game started!");

    game::context ctx{};
    // TODO: パラメータを外から渡せるようにする
    ctx.nnabla = std::make_unique<game::nnabla>(
        std::vector{"cpu"s}, "0"s,
        std::unordered_map<std::string, game::nnabla::nnp_file_type>{});

    model::refbox refbox{};
    std::unique_ptr<game::formation::base> formation{};

    std::chrono::steady_clock::time_point prev_time{};

    for (;;) {
      try {
        std::unique_lock lock{mutex_};
        // 前回の処理開始から cycle 待つ
        if (cv_.wait_until(lock, prev_time + cycle, [this] { return !running_; })) {
          break; // その間に stop() されたらループを抜ける
        }

        const auto current_time = std::chrono::steady_clock::now();

        const auto prev_cmd = refbox.command();

        ctx.team_color = team_color_;
        ctx.world      = updater_world_.value();
        refbox         = (is_global_refbox_ ? updater_refbox1_ : updater_refbox2_).value();

        const auto current_cmd = refbox.command();

        if (current_cmd != prev_cmd) {
          if (current_cmd == model::refbox::game_command::stop) {
            driver_.set_velocity_limit(velocity_limit_at_stopgame);
          } else {
            driver_.set_velocity_limit(std::numeric_limits<double>::max());
          }
        }

        if (!formation || need_reset_) {
          formation =
              std::make_unique<game::formation::first_formation>(ctx, refbox, active_robots_);
          need_reset_ = false;
          l_.info("formation resetted");
        }

        auto agents = formation->execute();
        for (auto agent : agents) {
          auto actions = agent->execute();
          for (auto action : actions) {
            auto command = action->execute();
            driver_.update_command(command);
          }
        }

        prev_time = current_time;
      } catch (const std::exception& e) {
        l_.error(fmt::format("exception at game_thread\n\t{}", e.what()));
      } catch (...) {
        l_.error("unknown exception at game_thread");
      }
    }

    // ロボットを全て停止させる
    {
      std::unique_lock lock{mutex_};
      for (const auto& id : active_robots_) {
        driver_.update_command(model::command{id});
      }
    }

    l_.info("game stopped!");
  }

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::atomic<bool> running_;

  bool is_global_refbox_;
  // formation のリセットが必要か
  bool need_reset_;

  model::team_color team_color_;
  model::updater::world& updater_world_;
  model::updater::refbox& updater_refbox1_;
  model::updater::refbox& updater_refbox2_;
  std::vector<unsigned int> active_robots_;

  ai_server::driver& driver_;

  std::shared_ptr<radio::base::command> radio_;

  std::thread game_thread_;

  logger::logger_for<game_runner> l_;
};

// game_runner などを設定するパネル
// --------------------------------
class game_panel final : public Gtk::VBox {
public:
  game_panel(model::updater::world& world, game_runner& runner)
      : updater_world_(world), runner_(runner) {
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
      this->pack_start(color_, Gtk::PACK_SHRINK, 4);

      dir_.set_label("Attack direction");
      dir_r1_.set_label("Right");
      dir_r2_.set_label("Left");
      dir_r2_.join_group(dir_r1_);
      dir_box_.set_border_width(4);
      dir_box_.pack_start(dir_r1_);
      dir_box_.pack_start(dir_r2_);
      dir_.add(dir_box_);
      this->pack_start(dir_, Gtk::PACK_SHRINK, 4);

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
          sigc::mem_fun(*this, &game_panel::handle_change_active_robots));
      robots_box_.pack_start(robots_id_box_);
      robots_box_.pack_start(apply_);
      robots_.add(robots_box_);
      this->pack_start(robots_, Gtk::PACK_SHRINK, 4);

      camera_.set_label("Camera");
      camera_id_box_.set_border_width(4);
      camera_id_box_.set_max_children_per_line(6);
      for (auto i = 0u; i < num_cameras; ++i) {
        camera_cb_.emplace_back(std::to_string(i));
        camera_id_box_.add(camera_cb_.back());
      }
      camera_.add(camera_id_box_);
      this->pack_start(camera_, Gtk::PACK_SHRINK, 4);

      refbox_.set_label("Refbox");
      refbox_r1_.set_label("Global");
      refbox_r2_.set_label("Local");
      refbox_r2_.join_group(refbox_r1_);
      refbox_box_.set_border_width(4);
      refbox_box_.pack_start(refbox_r1_);
      refbox_box_.pack_start(refbox_r2_);
      refbox_.add(refbox_box_);
      this->pack_start(refbox_, Gtk::PACK_SHRINK, 4);

      start_.set_label("start");
      start_.set_sensitive(false);
      start_.signal_clicked().connect(sigc::mem_fun(*this, &game_panel::handle_start_stop));
      this->pack_end(start_, Gtk::PACK_SHRINK);

      this->set_size_request(320, -1);
    }

    init_radio_buttons();
    init_check_buttons();
  }

  void set_ready() {
    start_.set_sensitive(true);
  }

private:
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
          sigc::mem_fun(*this, &game_panel::handle_active_robots_changed));
    }

    for (auto i = 0u; i < num_cameras; ++i) {
      auto& cb = camera_cb_.at(i);
      cb.set_active(updater_world_.is_camera_enabled(i));
      cb.signal_toggled().connect([id = i, this] {
        auto& cb = camera_cb_.at(id);
        if (cb.get_active()) {
          updater_world_.enable_camera(id);
          l_.info(fmt::format("camera{} enabled", id));
        } else {
          updater_world_.disable_camera(id);
          l_.info(fmt::format("camera{} disabled", id));
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

  Gtk::Frame color_, dir_, robots_, refbox_, camera_;
  Gtk::VBox robots_box_;
  Gtk::HBox color_box_, dir_box_, refbox_box_;
  Gtk::RadioButton color_r1_, color_r2_, dir_r1_, dir_r2_, refbox_r1_, refbox_r2_;
  std::vector<Gtk::CheckButton> robots_cb_, camera_cb_;
  Gtk::FlowBox robots_id_box_, camera_id_box_;
  Gtk::Button apply_, start_;

  model::updater::world& updater_world_;
  game_runner& runner_;

  logger::logger_for<game_panel> l_;
};

// 各種ステータスを表示するパネル
class status_tree final : public Gtk::TreeView {
public:
  status_tree() : store_{Gtk::TreeStore::create(columns_)} {
    this->set_model(store_);
    this->append_column("Name", columns_.name);
    this->append_column("Value", columns_.value);
    this->set_enable_tree_lines();
  }

  struct model final : public Gtk::TreeModel::ColumnRecord {
    model() {
      add(name);
      add(value);
    }
    Gtk::TreeModelColumn<Glib::ustring> name;
    Gtk::TreeModelColumn<Glib::ustring> value;
  };

  template <class T, class = void>
  struct handler;

  template <class, class = void>
  struct has_handler : std::false_type {};

  template <class T>
  struct has_handler<
      T, std::void_t<decltype(std::string{handler<T>::category_name}),
                     decltype(std::declval<const handler<T>>().update(std::declval<T>())),
                     std::enable_if_t<std::is_constructible_v<handler<T>, model,
                                                              std::function<Gtk::TreeRow()>>>>>
      : std::true_type {};

  template <class T>
  auto add(const std::string& name, const T& value, int interval = 1000)
      -> std::enable_if_t<has_handler<T>::value> {
    auto category       = *new_or_existing_row(handler<T>::category_name);
    auto base           = *store_->append(category.children());
    base[columns_.name] = name;

    auto h = handler<T>{columns_, [&] { return *store_->append(base.children()); }};
    Glib::signal_timeout().connect(
        [h, &value] {
          h.update(value);
          return true;
        },
        interval);
  }

private:
  Gtk::TreeModel::iterator new_or_existing_row(const std::string& name) {
    if (auto it = rows_.find(name); it != rows_.end()) {
      return it->second;
    }
    auto r = store_->append();
    rows_.insert({name, r});
    (*r)[columns_.name] = name;
    return r;
  }

  model columns_;
  std::unordered_map<std::string, Gtk::TreeModel::iterator> rows_;
  Glib::RefPtr<Gtk::TreeStore> store_;
};

template <class T>
struct status_tree::handler<T, std::void_t<decltype(std::declval<T>().total_messages()),
                                           decltype(std::declval<T>().messages_per_second()),
                                           decltype(std::declval<T>().parse_error()),
                                           decltype(std::declval<T>().last_updated())>> {
  static constexpr auto category_name = "Receivers";

  template <class F>
  handler(const status_tree::model& m, F add_row)
      : model{m}, r1{add_row()}, r2{add_row()}, r3{add_row()}, r4{add_row()} {
    r1[model.name] = "Total received messages";
    r2[model.name] = "Messages per second";
    r3[model.name] = "Parse error";
    r4[model.name] = "Last updated time";
  }

  void update(const T& receiver) const {
    const auto lu = receiver.last_updated();
    const auto tt = std::chrono::system_clock::to_time_t(lu);
    const auto e  = lu.time_since_epoch();
    const auto ms = (e - std::chrono::duration_cast<std::chrono::seconds>(e)) / 1ms;

    r1[model.value] = fmt::format("{}", receiver.total_messages());
    r2[model.value] = fmt::format("{}", receiver.messages_per_second());
    r3[model.value] = fmt::format("{}", receiver.parse_error());
    r4[model.value] = fmt::format("{:%T}.{:03d}", *std::localtime(&tt), ms);
  }

  const status_tree::model& model;
  Gtk::TreeRow r1, r2, r3, r4;
};

template <template <class> class T, class C>
struct status_tree::handler<T<C>, std::void_t<decltype(std::declval<T<C>>().connection()),
                                              decltype(std::declval<C>().total_messages()),
                                              decltype(std::declval<C>().messages_per_second()),
                                              decltype(std::declval<C>().total_errors()),
                                              decltype(std::declval<C>().last_sent())>> {
  static constexpr auto category_name = "Radio";

  template <class F>
  handler(const status_tree::model& m, F add_row)
      : model{m}, r1{add_row()}, r2{add_row()}, r3{add_row()}, r4{add_row()} {
    r1[model.name] = "Total sent messages";
    r2[model.name] = "Messages per second";
    r3[model.name] = "Total errors";
    r4[model.name] = "Last sent time";
  }

  void update(const T<C>& radio) const {
    const auto& c = radio.connection();
    const auto lu = c.last_sent();
    const auto tt = std::chrono::system_clock::to_time_t(lu);
    const auto e  = lu.time_since_epoch();
    const auto ms = (e - std::chrono::duration_cast<std::chrono::seconds>(e)) / 1ms;

    r1[model.value] = fmt::format("{}", c.total_messages());
    r2[model.value] = fmt::format("{}", c.messages_per_second());
    r3[model.value] = fmt::format("{}", c.total_errors());
    r4[model.value] = fmt::format("{:%T}.{:03d}", *std::localtime(&tt), ms);
  }

  const status_tree::model& model;
  Gtk::TreeRow r1, r2, r3, r4;
};

auto main(int argc, char** argv) -> int {
  logger::sink::ostream sink(std::cout, "{elapsed} {level:<5} {zone}: {message}");
  logger::logger l{"main()"};

  l.info("(⋈◍＞◡＜◍)。✧♡");

  boost::asio::io_context receiver_io{1}, driver_io{1};
  std::thread io_thread{}, driver_thread{};

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
      l.info("va filter (robot): "s + (use_va_filter ? "enabled"s : "disabled"s));
      // ボールの状態オブザーバを使うか
      if (use_ball_observer) {
        updater_world.ball_updater().set_filter<filter::state_observer::ball>(
            model::ball{}, std::chrono::system_clock::now());
      }
      l.info("state observer (ball): "s + (use_ball_observer ? "enabled"s : "disabled"s));
    }

    // Vision receiverの設定
    std::atomic<bool> vision_received{false};
    receiver::vision vision{receiver_io, "0.0.0.0", vision_address, vision_port};
    vision.on_receive([&updater_world, &vision_received, &l](auto&& p) {
      if (!vision_received) {
        // 最初に受信したときにメッセージを表示する
        l.info("vision packet received!");
        vision_received = true;
      }
      updater_world.update(std::forward<decltype(p)>(p));
    });
    l.info(fmt::format("vision: {}:{}", vision_address, vision_port));

    // Refbox receiverの設定
    std::atomic<bool> refbox1_received{false};
    model::updater::refbox updater_refbox1_{};
    receiver::refbox refbox1{receiver_io, "0.0.0.0", global_refbox_address, global_refbox_port};
    refbox1.on_receive([&updater_refbox1_, &refbox1_received, &l](auto&& p) {
      if (!refbox1_received) {
        // 最初に受信したときにメッセージを表示する
        l.info("refbox (global) packet received!");
        refbox1_received = true;
      }

      updater_refbox1_.update(std::forward<decltype(p)>(p));
    });
    l.info(fmt::format("global refbox: {}:{}", global_refbox_address, global_refbox_port));

    std::atomic<bool> refbox2_received{false};
    model::updater::refbox updater_refbox2_{};
    receiver::refbox refbox2{receiver_io, "0.0.0.0", local_refbox_address, local_refbox_port};
    refbox2.on_receive([&updater_refbox2_, &refbox2_received, &l](auto&& p) {
      if (!refbox2_received) {
        // 最初に受信したときにメッセージを表示する
        l.info("refbox (local) packet received!");
        refbox2_received = true;
      }

      updater_refbox2_.update(std::forward<decltype(p)>(p));
    });
    l.info(fmt::format("local refbox: {}:{}", local_refbox_address, local_refbox_port));

    // receiver_ioに登録されたタスクを別スレッドで開始
    io_thread = std::thread{[&receiver_io, &l] {
      try {
        receiver_io.run();
      } catch (std::exception& e) {
        l.error(fmt::format("exception at io_thread: {}", e.what()));
      }
    }};
    util::set_thread_name(io_thread, "io_thread");

    // Radioの設定
    auto radio = [&] {
      if constexpr (is_grsim) {
        auto con = std::make_unique<radio::connection::udp>(
            driver_io, boost::asio::ip::udp::endpoint{
                           boost::asio::ip::make_address(grsim_address), grsim_command_port});
        l.info(fmt::format("radio: grSim ({}:{})", grsim_address, grsim_command_port));
        return std::make_shared<radio::grsim<radio::connection::udp>>(std::move(con));
      } else {
        auto con = std::make_unique<radio::connection::serial>(
            driver_io, xbee_path, radio::connection::serial::baud_rate(57600));
        l.info(fmt::format("radio: kiks ({})", xbee_path));
        return std::make_shared<radio::kiks<radio::connection::serial>>(std::move(con));
      }
    }();

    // driver による命令の送信を別スレッドで開始
    ai_server::driver driver{driver_io, cycle, updater_world, model::team_color::yellow};
    driver_thread = std::thread{[&driver_io, &l] {
      try {
        driver_io.run();
      } catch (std::exception& e) {
        l.error(fmt::format("exception at driver_thread: {}", e.what()));
      }
    }};
    util::set_thread_name(driver_thread, "driver_thread");

    auto app = Gtk::Application::create(argc, argv);

    game_runner runner{updater_world, updater_refbox1_, updater_refbox2_, driver, radio};
    game_panel gp{updater_world, runner};

    Glib::signal_timeout().connect(
        [&vision_received, &gp, &l] {
          // Visionから値が取れてなかったら待つ
          if (!vision_received) return true;

          // 状態オブザーバなどの値が収束するまでもう少し待つ
          Glib::signal_timeout().connect_seconds_once(
              [&gp, &l] {
                l.info("ready!");
                gp.set_ready();
              },
              5);
          return false;
        },
        500);

    status_tree tree{};
    tree.add("Vision", vision);
    tree.add("Global RefBox", refbox1);
    tree.add("Local RefBox", refbox2);
    tree.add(is_grsim ? "grSim" : "KIKS", *radio);
    tree.expand_all();

    auto win = Gtk::Window{};
    auto box = Gtk::HBox{};
    {
      win.set_border_width(10);
      win.set_default_size(800, 500);
      win.set_title("Game configurations");

      box.pack_start(gp, Gtk::PACK_SHRINK, 10);
      box.pack_end(tree, Gtk::PACK_EXPAND_WIDGET, 10);

      win.add(box);
      win.show_all_children();
    }
    app->run(win);

    receiver_io.stop();
    driver_io.stop();
    io_thread.join();
    driver_thread.join();
  } catch (std::exception& e) {
    l.error(e.what());
    receiver_io.stop();
    driver_io.stop();
    io_thread.join();
    driver_thread.join();
    std::quick_exit(-1);
  } catch (...) {
    l.error("unknown error occurred");
    receiver_io.stop();
    driver_io.stop();
    io_thread.join();
    driver_thread.join();
    std::quick_exit(-1);
  }
}
