#define BOOST_TEST_DYN_LINK

#include <chrono>
#include <stdexcept>
#include <boost/asio.hpp>
#include <boost/test/unit_test.hpp>

#include "ai_server/driver.h"
#include "ai_server/model/updater/world.h"

BOOST_AUTO_TEST_SUITE(driver)

BOOST_AUTO_TEST_CASE(register_unregister) {
  boost::asio::io_service io_service{};
  ai_server::model::updater::world wu{};
  ai_server::driver d{io_service, std::chrono::seconds{1}, wu};

  ai_server::model::command cmd0{0};
  ai_server::model::command cmd1{1};

  // ロボットが登録されていない状態でupdate_command()を呼ぶとエラー
  BOOST_CHECK_THROW(d.update_command(true, cmd0), std::runtime_error);
  BOOST_CHECK_THROW(d.update_command(false, cmd1), std::runtime_error);

  d.register_robot(true, 0, nullptr, nullptr);
  BOOST_CHECK_NO_THROW(d.update_command(true, cmd0));
  BOOST_CHECK_THROW(d.update_command(false, cmd1), std::runtime_error);

  d.register_robot(false, 1, nullptr, nullptr);
  BOOST_CHECK_NO_THROW(d.update_command(true, cmd0));
  BOOST_CHECK_NO_THROW(d.update_command(false, cmd1));

  // 登録したロボットが正常に削除されるか
  d.unregister_robot(true, 0);
  BOOST_CHECK_THROW(d.update_command(true, cmd0), std::runtime_error);

  d.unregister_robot(false, 1);
  BOOST_CHECK_THROW(d.update_command(false, cmd1), std::runtime_error);
}

BOOST_AUTO_TEST_SUITE_END()
