#define BOOST_TEST_DYN_LINK

#include <sstream>
#include <boost/test/unit_test.hpp>

#include "ai_server/logger/formatter.h"
#include "ai_server/logger/log_item.h"
#include "ai_server/logger/logger.h"
#include "ai_server/logger/sink_registry.h"

#include "ai_server/logger/sink/function.h"
#include "ai_server/logger/sink/null.h"
#include "ai_server/logger/sink/ostream.h"

namespace l = ai_server::logger;

BOOST_AUTO_TEST_SUITE(logger)

BOOST_AUTO_TEST_CASE(log_level) {
  BOOST_TEST(l::log_level_to_string(l::log_level::debug) == "debug");
  BOOST_TEST(l::log_level_to_string(l::log_level::error) == "error");
  BOOST_TEST(l::log_level_to_string(l::log_level::info) == "info");
  BOOST_TEST(l::log_level_to_string(l::log_level::trace) == "trace");
  BOOST_TEST(l::log_level_to_string(l::log_level::warn) == "warn");

  BOOST_TEST(l::log_level_to_symbol(l::log_level::debug) == "[DEBUG]");
  BOOST_TEST(l::log_level_to_symbol(l::log_level::error) == "[-]");
  BOOST_TEST(l::log_level_to_symbol(l::log_level::info) == "[+]");
  BOOST_TEST(l::log_level_to_symbol(l::log_level::trace) == "[TRACE]");
  BOOST_TEST(l::log_level_to_symbol(l::log_level::warn) == "[!]");
}

BOOST_AUTO_TEST_CASE(log_item) {
  {
    l::log_item i{};
    i.level = l::log_level::trace;
    BOOST_TEST(l::format("{level}", i) == "trace");
    BOOST_TEST(l::format("{level_simple}", i) == "[TRACE]");
    i.level = l::log_level::debug;
    BOOST_TEST(l::format("{level}", i) == "debug");
    BOOST_TEST(l::format("{level_simple}", i) == "[DEBUG]");
    i.level = l::log_level::info;
    BOOST_TEST(l::format("{level}", i) == "info");
    BOOST_TEST(l::format("{level_simple}", i) == "[+]");
    i.level = l::log_level::warn;
    BOOST_TEST(l::format("{level}", i) == "warn");
    BOOST_TEST(l::format("{level_simple}", i) == "[!]");
    i.level = l::log_level::error;
    BOOST_TEST(l::format("{level}", i) == "error");
    BOOST_TEST(l::format("{level_simple}", i) == "[-]");

    BOOST_TEST(l::format("{level:<10}", i) == "error     ");
  }

  {
    l::log_item i{};
    i.message = "hello";
    BOOST_TEST(l::format("{message}", i) == "hello");
    BOOST_TEST(l::format("{message:<10}", i) == "hello     ");
  }

  {
    l::log_item i{};
    i.thread_id = 12;
    BOOST_TEST(l::format("{thread_id}", i) == "12");
    BOOST_TEST(l::format("{thread_id:x}", i) == "c");
  }

  {
    l::log_item i{};
    i.zone_name = "world";
    BOOST_TEST(l::format("{zone}", i) == "world");
    BOOST_TEST(l::format("{zone:<10}", i) == "world     ");
  }
}

BOOST_AUTO_TEST_CASE(sink_ostream) {
  {
    std::ostringstream s{};
    l::sink::ostream o(s, "{message}");

    const auto m = l::sink::levels_map_type{{"*", l::log_level::info}};
    BOOST_TEST(o.levels_map() == m);

    l::log_item i{};
    i.level     = l::log_level::info;
    i.message   = "hello";
    i.zone_name = "z";
    o.check_and_do_log(i);

    BOOST_TEST(s.str() == "hello\n");
  }

  {
    std::ostringstream s;
    l::sink::ostream o(s, "{level}", l::sink::levels_map_type{{"*", l::log_level::warn}});

    l::log_item i{};
    i.zone_name = "z";

    i.level = l::log_level::info;
    o.check_and_do_log(i);

    i.level = l::log_level::warn;
    o.check_and_do_log(i);

    i.level = l::log_level::error;
    o.check_and_do_log(i);

    BOOST_TEST(s.str() == "warn\nerror\n");
  }

  {
    std::ostringstream s;
    l::sink::ostream o(s, "{zone} {level}",
                       l::sink::levels_map_type{{"nyan", l::log_level::info}});

    l::log_item i1{};
    i1.level     = l::log_level::info;
    i1.message   = "m";
    i1.zone_name = "myon";
    o.check_and_do_log(i1);

    l::log_item i2{};
    i2.level     = l::log_level::info;
    i2.message   = "m";
    i2.zone_name = "nyan";
    o.check_and_do_log(i2);

    BOOST_TEST(s.str() == "nyan info\n");
  }
}

BOOST_AUTO_TEST_CASE(sink_function) {
  l::log_item last_item1{};
  l::sink::function f1([&last_item1](auto&& item) { last_item1 = item; });
  const auto m1 = l::sink::levels_map_type{{"*", l::log_level::info}};
  BOOST_TEST(f1.levels_map() == m1);

  l::log_item last_item2{};
  l::sink::function f2({{"*", l::log_level::error}},
                       [&last_item2](auto&& item) { last_item2 = item; });
  const auto m2 = l::sink::levels_map_type{{"*", l::log_level::error}};
  BOOST_TEST(f2.levels_map() == m2);

  {
    l::log_item i1{};
    i1.level     = l::log_level::error;
    i1.message   = "i1";
    i1.zone_name = "z";
    f1.check_and_do_log(i1);
    f2.check_and_do_log(i1);
  }

  BOOST_TEST(last_item1.message == "i1");
  BOOST_TEST(last_item2.message == "i1");

  {
    l::log_item i2{};
    i2.level     = l::log_level::info;
    i2.message   = "i2";
    i2.zone_name = "z";
    f1.check_and_do_log(i2);
    f2.check_and_do_log(i2);
  }

  BOOST_TEST(last_item1.message == "i2");
  BOOST_TEST(last_item2.message == "i1");
}

BOOST_AUTO_TEST_CASE(sink_registry1) {
  std::ostringstream s1{};
  std::ostringstream s2{};

  l::sink::ostream o(s1, "{message}");

  {
    l::log_item i{};
    i.level     = l::log_level::info;
    i.message   = "1";
    i.zone_name = "z";
    l::sink_registry::global_sink_registry().notify_all(i);
  }

  BOOST_TEST(s1.str() == "1\n");
  BOOST_TEST(s2.str() == "");

  {
    l::sink::ostream o(s2, "{message}");
    l::log_item i{};
    i.level     = l::log_level::info;
    i.message   = "2";
    i.zone_name = "z";
    l::sink_registry::global_sink_registry().notify_all(i);
  }

  BOOST_TEST(s1.str() == "1\n2\n");
  BOOST_TEST(s2.str() == "2\n");

  {
    l::log_item i{};
    i.level     = l::log_level::info;
    i.message   = "3";
    i.zone_name = "z";
    l::sink_registry::global_sink_registry().notify_all(i);
  }

  BOOST_TEST(s1.str() == "1\n2\n3\n");
  BOOST_TEST(s2.str() == "2\n");
}

BOOST_AUTO_TEST_CASE(sink_registry2) {
  {
    // sinkのmoveが正しく動作するか
    std::ostringstream s{};
    auto o1 = l::sink::ostream(s, "{message}");

    {
      l::log_item i{};
      i.level     = l::log_level::info;
      i.message   = "1";
      i.zone_name = "z";
      l::sink_registry::global_sink_registry().notify_all(i);
    }

    auto o2 = std::move(o1);

    {
      l::log_item i{};
      i.level     = l::log_level::info;
      i.message   = "2";
      i.zone_name = "z";
      l::sink_registry::global_sink_registry().notify_all(i);
    }

    BOOST_TEST(s.str() == "1\n2\n");
  }
}

BOOST_AUTO_TEST_CASE(sink_registry3) {
  auto& r = l::sink_registry::global_sink_registry();

  l::sink::null n{};

  // sink の 2重登録はエラー
  BOOST_CHECK_THROW(r.register_sink(&n), std::runtime_error);
}

BOOST_AUTO_TEST_CASE(logger) {
  std::ostringstream s1{};
  l::sink::ostream o1(s1, "{zone} {message}");

  l::logger l1("z1");

  {
    std::ostringstream s2{};
    l::sink::ostream o2(s2, "{zone} {message}");

    l1.info("111");

    l::logger l2("z2");
    l2.info("222");

    BOOST_TEST(s2.str() == "z1 111\nz2 222\n");
  }

  l1.info("333");

  BOOST_TEST(s1.str() == "z1 111\nz2 222\nz1 333\n");
}

BOOST_AUTO_TEST_SUITE_END()
