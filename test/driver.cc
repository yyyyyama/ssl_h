#define BOOST_TEST_DYN_LINK

#include <chrono>
#include <memory>
#include <stdexcept>
#include <vector>
#include <boost/asio.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/test/unit_test.hpp>

#include "ai_server/controller/base.h"
#include "ai_server/driver.h"
#include "ai_server/model/team_color.h"
#include "ai_server/model/updater/world.h"
#include "ai_server/radio/base/base.h"

#include "ssl-protos/vision_wrapper.pb.h"

using namespace std::chrono_literals;
namespace controller = ai_server::controller;
namespace model      = ai_server::model;
namespace radio      = ai_server::radio;

BOOST_TEST_DONT_PRINT_LOG_VALUE(model::team_color)

BOOST_AUTO_TEST_SUITE(driver)

BOOST_AUTO_TEST_CASE(set_team_color) {
  boost::asio::io_service io_service{};
  ai_server::model::updater::world wu{};

  {
    // コンストラクタでチームカラーが正しく設定されているか
    ai_server::driver d{io_service, 1s, wu, model::team_color::blue};
    BOOST_TEST(d.team_color() == model::team_color::blue);

    // チームカラーが変更できるか
    d.set_team_color(model::team_color::yellow);
    BOOST_TEST(d.team_color() == model::team_color::yellow);
  }

  {
    ai_server::driver d{io_service, 1s, wu, model::team_color::yellow};
    BOOST_TEST(d.team_color() == model::team_color::yellow);

    d.set_team_color(model::team_color::blue);
    BOOST_TEST(d.team_color() == model::team_color::blue);
  }
}

BOOST_AUTO_TEST_CASE(register_robot) {
  boost::asio::io_service io_service{};
  ai_server::model::updater::world wu{};
  ai_server::driver d{io_service, std::chrono::seconds{1}, wu, model::team_color::yellow};

  // ロボットが登録されていない状態でupdate_command()を呼ぶとエラー
  BOOST_CHECK_THROW(d.update_command(0, {}), std::runtime_error);
  BOOST_CHECK_THROW(d.update_command(1, {}), std::runtime_error);

  // まだ登録されていない
  BOOST_TEST(!d.registered(0));
  BOOST_TEST(!d.registered(1));

  // ID0のロボットを登録
  BOOST_CHECK_NO_THROW(d.register_robot(0, nullptr, nullptr));
  BOOST_CHECK_NO_THROW(d.update_command(0, {}));
  BOOST_TEST(d.registered(0));

  // ID1のロボットを登録
  BOOST_CHECK_NO_THROW(d.register_robot(1, nullptr, nullptr));
  BOOST_CHECK_NO_THROW(d.update_command(1, {}));
  BOOST_TEST(d.registered(1));

  // 登録したロボットが正常に削除されるか
  d.unregister_robot(0);
  BOOST_CHECK_THROW(d.update_command(0, {}), std::runtime_error);
  BOOST_CHECK_NO_THROW(d.update_command(1, {}));
  BOOST_TEST(!d.registered(0));
  d.unregister_robot(1);
  BOOST_CHECK_THROW(d.update_command(1, {}), std::runtime_error);
  BOOST_TEST(!d.registered(1));
}

struct mock_controller : public controller::base {
  bool is_stable() const {
    return stable_flag_;
  }

  bool executed_ = false;

  model::field field_;

protected:
  controller::base::result_type update(const model::robot&, const model::field& field,
                                       const model::setpoint::position&,
                                       const model::setpoint::angle&) {
    executed_ = true;
    field_    = field;
    return {};
  }
  controller::base::result_type update(const model::robot&, const model::field& field,
                                       const model::setpoint::position&,
                                       const model::setpoint::velangular&) {
    executed_ = true;
    field_    = field;
    return {};
  }
  controller::base::result_type update(const model::robot&, const model::field& field,
                                       const model::setpoint::velocity&,
                                       const model::setpoint::angle&) {
    executed_ = true;
    field_    = field;
    return {};
  }
  controller::base::result_type update(const model::robot&, const model::field& field,
                                       const model::setpoint::velocity& v,
                                       const model::setpoint::velangular& va) {
    executed_ = true;
    field_    = field;
    return {std::get<0>(v), std::get<1>(v), std::get<0>(va)};
  }
};

struct mock_radio : public radio::base::command {
  std::vector<
      std::tuple<unsigned int, double, double, double, std::shared_ptr<model::motion::base>>>
      commands_;
  model::team_color color_;

  void send(model::team_color color, unsigned int id, const model::command::kick_flag_t&, int,
            double vx, double vy, double omega) {
    commands_.emplace_back(id, vx, vy, omega, nullptr);
    color_ = color;
  }

  void send(model::team_color color, unsigned int id,
            std::shared_ptr<model::motion::base> motion) {
    commands_.emplace_back(id, 0.0, 0.0, 0.0, motion);
    color_ = color;
  }
};

struct command_updated_handler {
  std::vector<
      std::tuple<unsigned int, double, double, double, std::shared_ptr<model::motion::base>>>
      commands;
  model::team_color color;

  void operator()(model::team_color c, unsigned int id, const model::command::kick_flag_t&, int,
                  double vx, double vy, double omega,
                  std::shared_ptr<model::motion::base> motion = nullptr) {
    commands.emplace_back(id, vx, vy, omega, motion);
    color = c;
  }
};

BOOST_AUTO_TEST_CASE(velocity_limit) {
  boost::asio::io_service io_service{};
  ai_server::model::updater::world wu{};
  ai_server::driver d{io_service, std::chrono::seconds{1}, wu, model::team_color::blue};

  // 適当なControllerをいくつか初期化し登録する
  // 後からアクセスできるように参照を残しておく
  auto c1_ptr    = std::make_unique<mock_controller>();
  auto c2_ptr    = std::make_unique<mock_controller>();
  const auto& c1 = *c1_ptr;
  const auto& c2 = *c2_ptr;
  d.register_robot(1, std::move(c1_ptr), nullptr);
  d.register_robot(2, std::move(c2_ptr), nullptr);

  // 変更前
  BOOST_TEST(c1.velocity_limit() == std::numeric_limits<double>::max());
  BOOST_TEST(c2.velocity_limit() == std::numeric_limits<double>::max());

  // 変更してみる
  d.set_velocity_limit(123);
  BOOST_TEST(c1.velocity_limit() == 123);
  BOOST_TEST(c2.velocity_limit() == 123);

  // さらに変更してみる
  d.set_velocity_limit(456);
  BOOST_TEST(c1.velocity_limit() == 456);
  BOOST_TEST(c2.velocity_limit() == 456);

  // もとに戻してみる
  d.set_velocity_limit(std::numeric_limits<double>::max());
  BOOST_TEST(c1.velocity_limit() == std::numeric_limits<double>::max());
  BOOST_TEST(c2.velocity_limit() == std::numeric_limits<double>::max());
}

BOOST_AUTO_TEST_CASE(set_stable) {
  boost::asio::io_service io_service{};
  ai_server::model::updater::world wu{};
  ai_server::driver d{io_service, 100us, wu, model::team_color::blue};

  auto c1_ptr    = std::make_unique<mock_controller>();
  const auto& c1 = *c1_ptr;
  d.register_robot(1, std::move(c1_ptr), nullptr);

  auto c2_ptr    = std::make_unique<mock_controller>();
  const auto& c2 = *c2_ptr;
  d.register_robot(2, std::move(c2_ptr), nullptr);

  d.set_stable(true);
  BOOST_TEST(c1.is_stable());
  BOOST_TEST(c2.is_stable());

  d.set_stable(false);
  BOOST_TEST(!c1.is_stable());
  BOOST_TEST(!c2.is_stable());
}

BOOST_AUTO_TEST_CASE(main_loop) {
  boost::asio::io_service io_service{};
  ai_server::model::updater::world wu{};
  ai_server::driver d{io_service, 100us, wu, model::team_color::blue};

  auto c1_ptr = std::make_unique<mock_controller>();
  auto s1_ptr = std::make_unique<mock_radio>();
  auto& c1    = *c1_ptr;
  auto& s1    = *s1_ptr;
  d.register_robot(1, std::move(c1_ptr), std::move(s1_ptr));

  auto c2_ptr = std::make_unique<mock_controller>();
  auto s2_ptr = std::make_unique<mock_radio>();
  auto& c2    = *c2_ptr;
  auto& s2    = *s2_ptr;
  d.register_robot(2, std::move(c2_ptr), std::move(s2_ptr));

  command_updated_handler handler{};
  d.on_command_updated(std::ref(handler));

  // ロボットが検出されていないとき命令は送信されない
  io_service.run_one();
  BOOST_TEST(!c1.executed_);
  BOOST_TEST(s1.commands_.empty());
  BOOST_TEST(!c2.executed_);
  BOOST_TEST(s2.commands_.empty());
  BOOST_TEST(handler.commands.empty());

  // blue の ID 1 が見えるようにしてみる
  {
    ssl_protos::vision::Packet p{};

    auto md = p.mutable_detection();
    md->set_camera_id(0);

    auto r = md->add_robots_blue();
    r->set_robot_id(1);
    r->set_x(0);
    r->set_y(0);
    r->set_orientation(0);
    r->set_confidence(100);

    wu.update(p);
  }

  // ID 1 に対して命令が送られる
  // 命令の初期値は停止 (速度0)
  io_service.run_one();
  {
    BOOST_TEST(c1.executed_);
    BOOST_TEST(s1.commands_.size() == 1);
    const auto& [id, vx, vy, omega, motion] = s1.commands_.back();
    BOOST_TEST(id == 1);
    BOOST_TEST(vx == 0);
    BOOST_TEST(vy == 0);
    BOOST_TEST(omega == 0);
    // コンストラクタのteam_colorの設定が反映されている
    BOOST_TEST(s1.color_ == model::team_color::blue);
    c1.executed_ = false;
    s1.commands_.clear();
  }
  BOOST_TEST(!c2.executed_);
  BOOST_TEST(s2.commands_.empty());
  {
    BOOST_TEST(handler.commands.size() == 1);
    const auto& [id, vx, vy, omega, motion] = handler.commands.back();
    BOOST_TEST(id == 1);
    BOOST_TEST(vx == 0);
    BOOST_TEST(vy == 0);
    BOOST_TEST(omega == 0);
    BOOST_TEST(handler.color == model::team_color::blue);
    handler.commands.clear();
  }

  // チームカラーを yellow に変更
  d.set_team_color(model::team_color::yellow);

  // ロボットが検出されていないとき命令は送信されない
  io_service.run_one();
  BOOST_TEST(!c1.executed_);
  BOOST_TEST(s1.commands_.empty());
  BOOST_TEST(!c2.executed_);
  BOOST_TEST(s2.commands_.empty());
  BOOST_TEST(handler.commands.empty());

  // yellow の ID 1 が見えるようにしてみる
  {
    ssl_protos::vision::Packet p{};

    auto md = p.mutable_detection();
    md->set_camera_id(0);

    auto r = md->add_robots_yellow();
    r->set_robot_id(1);
    r->set_x(0);
    r->set_y(0);
    r->set_orientation(0);
    r->set_confidence(100);

    wu.update(p);
  }

  // ID 1 に対して命令が送られる
  io_service.run_one();
  {
    BOOST_TEST(c1.executed_);
    BOOST_TEST(s1.commands_.size() == 1);
    const auto& [id, vx, vy, omega, motion] = s1.commands_.back();
    BOOST_TEST(id == 1);
    BOOST_TEST(vx == 0);
    BOOST_TEST(vy == 0);
    BOOST_TEST(omega == 0);
    // set_team_colorでのteam_colorの変更が反映されている
    BOOST_TEST(s1.color_ == model::team_color::yellow);
    c1.executed_ = false;
    s1.commands_.clear();
  }
  BOOST_TEST(!c2.executed_);
  BOOST_TEST(s2.commands_.empty());
  {
    BOOST_TEST(handler.commands.size() == 1);
    const auto& [id, vx, vy, omega, motion] = handler.commands.back();
    BOOST_TEST(id == 1);
    BOOST_TEST(vx == 0);
    BOOST_TEST(vy == 0);
    BOOST_TEST(omega == 0);
    BOOST_TEST(handler.color == model::team_color::yellow);
    handler.commands.clear();
  }

  // ID 1 の命令を更新してみる
  {
    model::command c{};
    c.set_velocity({1, 2, 3});
    d.update_command(1, c);
  }

  // ID 1 に対して命令が送られる
  io_service.run_one();
  {
    BOOST_TEST(c1.executed_);
    BOOST_TEST(s1.commands_.size() == 1);
    const auto& [id, vx, vy, omega, motion] = s1.commands_.back();
    BOOST_TEST(id == 1);
    BOOST_TEST(vx == 0);
    BOOST_TEST(vy == 0);
    BOOST_TEST(omega == 0);
    c1.executed_ = false;
    s1.commands_.clear();
  }
  BOOST_TEST(!c2.executed_);
  BOOST_TEST(s2.commands_.empty());
  {
    BOOST_TEST(handler.commands.size() == 1);
    const auto& [id, vx, vy, omega, motion] = handler.commands.back();
    BOOST_TEST(id == 1);
    BOOST_TEST(vx == 1);
    BOOST_TEST(vy == 2);
    BOOST_TEST(omega == 3);
    BOOST_TEST(handler.color == model::team_color::yellow);
    handler.commands.clear();
  }

  // yellow の ID 2 も見えるようにしてみる
  {
    ssl_protos::vision::Packet p{};

    auto md = p.mutable_detection();
    md->set_camera_id(1);

    auto r = md->add_robots_yellow();
    r->set_robot_id(2);
    r->set_x(0);
    r->set_y(0);
    r->set_orientation(0);
    r->set_confidence(100);

    wu.update(p);
  }

  // ID 1, ID 2 に対して命令が送られる
  io_service.run_one();
  {
    BOOST_TEST(c1.executed_);
    BOOST_TEST(s1.commands_.size() == 1);
    const auto& [id, vx, vy, omega, motion] = s1.commands_.back();
    BOOST_TEST(id == 1);
    BOOST_TEST(vx == 0);
    BOOST_TEST(vy == 0);
    BOOST_TEST(omega == 0);
    c1.executed_ = false;
    s1.commands_.clear();
  }
  {
    BOOST_TEST(c2.executed_);
    BOOST_TEST(s2.commands_.size() == 1);
    const auto& [id, vx, vy, omega, motion] = s2.commands_.back();
    BOOST_TEST(id == 2);
    BOOST_TEST(vx == 0);
    BOOST_TEST(vy == 0);
    BOOST_TEST(omega == 0);
    c2.executed_ = false;
    s2.commands_.clear();
  }
  {
    BOOST_TEST(handler.color == model::team_color::yellow);
    BOOST_TEST(handler.commands.size() == 2);

    const auto& cmds = handler.commands;
    const auto& c1 =
        std::find_if(cmds.cbegin(), cmds.cend(), [](auto&& c) { return std::get<0>(c) == 1; });
    BOOST_TEST((c1 != cmds.cend()));
    BOOST_TEST(std::get<1>(*c1) == 1);
    BOOST_TEST(std::get<2>(*c1) == 2);
    BOOST_TEST(std::get<3>(*c1) == 3);

    const auto& c2 =
        std::find_if(cmds.cbegin(), cmds.cend(), [](auto&& c) { return std::get<0>(c) == 2; });
    BOOST_TEST((c2 != cmds.cend()));
    BOOST_TEST(std::get<1>(*c2) == 0);
    BOOST_TEST(std::get<2>(*c2) == 0);
    BOOST_TEST(std::get<3>(*c2) == 0);

    handler.commands.clear();
  }
}

BOOST_AUTO_TEST_CASE(main_loop2) {
  boost::asio::io_context ctx{};
  ai_server::model::updater::world wu{};
  ai_server::driver d{ctx, 100us, wu, model::team_color::blue};

  auto c1_ptr = std::make_unique<mock_controller>();
  auto s1_ptr = std::make_unique<mock_radio>();
  auto& c1    = *c1_ptr;
  auto& s1    = *s1_ptr;
  d.register_robot(1, std::move(c1_ptr), std::move(s1_ptr));

  {
    ssl_protos::vision::Packet p{};

    auto md = p.mutable_detection();
    md->set_camera_id(0);

    auto r = md->add_robots_blue();
    r->set_robot_id(1);
    r->set_x(0);
    r->set_y(0);
    r->set_orientation(0);
    r->set_confidence(100);

    auto mf = p.mutable_geometry()->mutable_field();
    mf->set_field_length(9000);
    mf->set_field_width(6000);

    wu.update(p);
  }

  ctx.run_one();
  {
    BOOST_TEST(c1.executed_);
    const auto& c = s1.commands_.back();
    BOOST_TEST(std::get<0>(c) == 1);

    // field が正しく渡されているか
    BOOST_TEST(c1.field_.length() == 9000);
    BOOST_TEST(c1.field_.width() == 6000);

    c1.executed_ = false;
    s1.commands_.clear();
  }

  // field の値を変えてみる
  {
    ssl_protos::vision::Packet p{};

    auto mf = p.mutable_geometry()->mutable_field();
    mf->set_field_length(18000);
    mf->set_field_width(12000);

    wu.update(p);
  }

  ctx.run_one();
  {
    BOOST_TEST(c1.executed_);
    const auto& c = s1.commands_.back();
    BOOST_TEST(std::get<0>(c) == 1);

    // field が正しく渡されているか
    BOOST_TEST(c1.field_.length() == 18000);
    BOOST_TEST(c1.field_.width() == 12000);

    c1.executed_ = false;
    s1.commands_.clear();
  }
}

BOOST_AUTO_TEST_CASE(main_loop3, *boost::unit_test::tolerance(0.0000001)) {
  namespace bmc = boost::math::constants;

  boost::asio::io_context ctx{};
  ai_server::model::updater::world wu{};
  ai_server::driver d{ctx, 100us, wu, model::team_color::blue};

  d.register_robot(1, std::make_unique<mock_controller>(), std::make_unique<mock_radio>());

  command_updated_handler handler{};
  d.on_command_updated(std::ref(handler));

  {
    ssl_protos::vision::Packet p{};
    auto md = p.mutable_detection();
    md->set_camera_id(0);

    auto r = md->add_robots_blue();
    r->set_robot_id(1);
    r->set_x(0);
    r->set_y(0);
    r->set_orientation(bmc::half_pi<float>());
    r->set_confidence(100);

    wu.update(p);
  }

  {
    model::command c{};
    c.set_velocity(1, 2, 3);
    d.update_command(1, c);
  }

  ctx.run_one();
  {
    BOOST_TEST(handler.commands.size() == 1);
    const auto& [id, vx, vy, omega, motion] = handler.commands.back();
    // on_command_updated() に渡される速度は、controller が出力するロボット基準の値を
    // フィールド基準にもどすためにロボットの角度だけ回転される
    BOOST_TEST(id == 1);
    BOOST_TEST(vx == -2.0);
    BOOST_TEST(vy == 1.0);
    BOOST_TEST(omega == 3.0);
    BOOST_TEST(handler.color == model::team_color::blue);
    handler.commands.clear();
  }
}

BOOST_AUTO_TEST_SUITE_END()
