#define BOOST_TEST_DYN_LINK

#include <chrono>
#include <stdexcept>
#include <boost/asio.hpp>
#include <boost/test/unit_test.hpp>

#include "ai_server/driver.h"
#include "ai_server/model/team_color.h"
#include "ai_server/model/updater/world.h"

using namespace std::chrono_literals;
namespace model = ai_server::model;

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

  ai_server::model::command cmd0{0};
  ai_server::model::command cmd1{1};

  // ロボットが登録されていない状態でupdate_command()を呼ぶとエラー
  BOOST_CHECK_THROW(d.update_command(cmd0), std::runtime_error);
  BOOST_CHECK_THROW(d.update_command(cmd1), std::runtime_error);

  // ID0のロボットを登録
  BOOST_CHECK_NO_THROW(d.register_robot(0, nullptr, nullptr));
  BOOST_CHECK_NO_THROW(d.update_command(cmd0));

  // ID1のロボットを登録
  BOOST_CHECK_NO_THROW(d.register_robot(1, nullptr, nullptr));
  BOOST_CHECK_NO_THROW(d.update_command(cmd1));

  // 登録したロボットが正常に削除されるか
  d.unregister_robot(0);
  BOOST_CHECK_THROW(d.update_command(cmd0), std::runtime_error);
  BOOST_CHECK_NO_THROW(d.update_command(cmd1));
  d.unregister_robot(1);
  BOOST_CHECK_THROW(d.update_command(cmd1), std::runtime_error);
}

BOOST_AUTO_TEST_SUITE_END()
