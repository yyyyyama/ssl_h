#define BOOST_TEST_DYN_LINK

#include <chrono>
#include <future>
#include <limits>
#include <random>
#include <string>
#include <thread>
#include <vector>
#include <boost/asio.hpp>
#include <boost/test/unit_test.hpp>

#include "ai_server/util/net/multicast/receiver.h"
#include "ai_server/util/net/multicast/sender.h"

#include "test_helpers/asio.h"

using namespace std::chrono_literals;
using namespace std::string_literals;
using namespace ai_server::util::net::multicast;

using promise_type = std::promise<std::tuple<receiver::buffer_t, std::size_t, std::uint64_t,
                                             std::chrono::system_clock::time_point>>;

BOOST_AUTO_TEST_SUITE(multicast)

BOOST_AUTO_TEST_CASE(receiver_error, *boost::unit_test::timeout(30)) {
  {
    boost::asio::io_context ctx{};

    // 文字列で渡したアドレスに不備があったら例外
    BOOST_CHECK_THROW(receiver(ctx, "123.456.789.012", "224.5.23.2", 10000),
                      boost::system::system_error);
    BOOST_CHECK_THROW(receiver(ctx, "0.0.0.0", "123.456.789.012", 10000),
                      boost::system::system_error);
  }

  {
    boost::asio::io_context ctx{};

    // 第2引数がマルチキャストアドレスではない
    receiver r{ctx, "0.0.0.0", "10.0.0.0", 10000};

    std::promise<boost::system::error_code> promise{};
    r.on_error([&promise](auto& error) { promise.set_value(error); });

    auto future = promise.get_future();

    // ソケットの細かな初期化はコルーチン内で行われるのでまだエラーは発生していない
    BOOST_TEST((future.wait_for(50ms) == std::future_status::timeout));

    auto t = run_io_context_in_new_thread(ctx);

    // エラー発生
    BOOST_TEST(future.get() == boost::asio::error::invalid_argument);
  }
}

BOOST_AUTO_TEST_CASE(send_and_receive, *boost::unit_test::timeout(30)) {
  boost::asio::io_context ctx{};

  // 受信クラスの初期化
  // listen_addr = 0.0.0.0, multicast_addr = 224.5.23.2, port = 10000
  receiver r{ctx, "0.0.0.0", "224.5.23.2", 10000};

  // 送信クラスの初期化
  // multicast_addr = 224.5.23.2, port = 10000
  sender s{ctx, "224.5.23.2", 10000};

  promise_type promise{};
  r.on_receive([&promise](auto&&... args) {
    promise.set_value({std::forward<decltype(args)>(args)...});
  });

  boost::system::error_code error{};
  r.on_error([&error](auto& e) { error = e; });

  // 受信を開始する
  auto t = run_io_context_in_new_thread(ctx);

  {
    // promise のリセット
    // 注: 通常のコーディングでは、別スレッドで変化する値の操作に十分注意すること
    promise = promise_type{};

    // std::string("Hello!") を送信
    const auto s1 = "Hello!"s;
    s.send(s1);

    // 受信したデータを取得
    const auto& result = promise.get_future().get();

    // エラーが発生していないか
    BOOST_TEST(!error);

    // 受信したデータが送信したものと一致するか
    BOOST_TEST(std::get<1>(result) == s1.length());
    const auto s2 = std::string(std::cbegin(std::get<0>(result)), std::get<1>(result));
    BOOST_TEST(s2 == s1);

    // 受信したメッセージの数がカウントアップしているか
    BOOST_TEST(std::get<2>(result) == 1);
  }

  {
    promise = promise_type{};

    // receiverのバッファサイズ分(4096byte)の文字列を送信
    const auto s1 = std::string(receiver::buffer_size / 4, 'a') +
                    std::string(receiver::buffer_size / 4, 'b') +
                    std::string(receiver::buffer_size / 4, 'c') +
                    std::string(receiver::buffer_size / 4, 'd');
    s.send(s1);

    // 受信したデータを取得
    const auto& result = promise.get_future().get();

    // エラーが発生していないか
    BOOST_TEST(!error);

    // 受信したデータが送信したものと一致するか
    BOOST_TEST(std::get<1>(result) == s1.length());
    const auto s2 = std::string(std::cbegin(std::get<0>(result)), std::get<1>(result));
    BOOST_TEST(s2 == s1);

    // 受信したメッセージの数がカウントアップしているか
    BOOST_TEST(std::get<2>(result) == 2);
  }
}

BOOST_AUTO_TEST_CASE(random_data1, *boost::unit_test::timeout(30)) {
  boost::asio::io_context ctx{};

  // 受信クラスの初期化
  // listen_addr = 0.0.0.0, multicast_addr = 224.5.23.2, port = 10001
  receiver r{ctx, "0.0.0.0", "224.5.23.2", 10001};

  // 送信クラスの初期化
  // multicast_addr = 224.5.23.2, port = 10001
  sender s{ctx, "224.5.23.2", 10001};

  promise_type promise{};
  r.on_receive([&promise](auto&&... args) {
    promise.set_value({std::forward<decltype(args)>(args)...});
  });

  boost::system::error_code error{};
  r.on_error([&error](auto& e) { error = e; });

  // 受信を開始する
  auto t = run_io_context_in_new_thread(ctx);

  // 乱数生成器を初期化
  std::mt19937 mt{0x12345678};

  for (auto i = 0u; i < 1024; ++i) {
    promise = promise_type{};

    using value_type = typename receiver::buffer_t::value_type;

    // ランダムな長さ[1, 4096]のランダムなデータの入った配列を生成
    const auto size = std::uniform_int_distribution<std::size_t>(1, receiver::buffer_size)(mt);
    const auto v1   = [&mt, size] {
      std::vector<value_type> v(size);
      std::uniform_int_distribution<value_type> d(std::numeric_limits<value_type>::min(),
                                                  std::numeric_limits<value_type>::max());
      std::generate(v.begin(), v.end(), [&mt, &d] { return d(mt); });
      return v;
    }();

    // 送信サイズを指定しないsend
    s.send(v1);

    // 受信したデータを取得
    const auto& result = promise.get_future().get();

    // エラーが発生していないか
    BOOST_TEST(!error);

    // 受信したデータが送信したものと一致するか
    BOOST_TEST(std::get<1>(result) == v1.size());
    const auto& buf = std::get<0>(result);
    const std::vector<value_type> v2(buf.cbegin(), buf.cbegin() + v1.size());
    BOOST_TEST(v2 == v1, boost::test_tools::per_element());

    // 受信したメッセージの数がカウントアップしているか
    BOOST_TEST(std::get<2>(result) == (i + 1));
  }
}

BOOST_AUTO_TEST_CASE(random_data2, *boost::unit_test::timeout(30)) {
  boost::asio::io_context ctx{};

  // 受信クラスの初期化
  // listen_addr = 0.0.0.0, multicast_addr = 224.5.23.2, port = 10002
  receiver r{ctx, "0.0.0.0", "224.5.23.2", 10002};

  // 送信クラスの初期化
  // multicast_addr = 224.5.23.2, port = 10002
  sender s{ctx, "224.5.23.2", 10002};

  promise_type promise{};
  r.on_receive([&promise](auto&&... args) {
    promise.set_value({std::forward<decltype(args)>(args)...});
  });

  boost::system::error_code error{};
  r.on_error([&error](auto& e) { error = e; });

  // 受信を開始する
  auto t = run_io_context_in_new_thread(ctx);

  // 乱数生成器を初期化
  std::mt19937 mt{0x12345678};

  for (auto i = 0u; i < 1024; ++i) {
    promise = promise_type{};

    using value_type = typename receiver::buffer_t::value_type;

    // ランダムな長さ[1, 4096]のランダムなデータの入った配列を生成
    const auto size = std::uniform_int_distribution<std::size_t>(1, receiver::buffer_size)(mt);
    const auto v1   = [&mt, size] {
      std::vector<value_type> v(size);
      std::uniform_int_distribution<value_type> d(std::numeric_limits<value_type>::min(),
                                                  std::numeric_limits<value_type>::max());
      std::generate(v.begin(), v.end(), [&mt, &d] { return d(mt); });
      return v;
    }();

    // 送信サイズを指定するsend
    s.send(v1, size);

    // 受信したデータを取得
    const auto& result = promise.get_future().get();

    // エラーが発生していないか
    BOOST_TEST(!error);

    // 受信したデータが送信したものと一致するか
    BOOST_TEST(std::get<1>(result) == v1.size());
    const auto& buf = std::get<0>(result);
    const std::vector<value_type> v2(buf.cbegin(), buf.cbegin() + v1.size());
    BOOST_TEST(v2 == v1, boost::test_tools::per_element());

    // 受信したメッセージの数がカウントアップしているか
    BOOST_TEST(std::get<2>(result) == (i + 1));
  }
}

BOOST_AUTO_TEST_CASE(messages_per_second, *boost::unit_test::timeout(30)) {
  boost::asio::io_context ctx{};

  // 受信クラスの初期化
  // listen_addr = 0.0.0.0, multicast_addr = 224.5.23.2, port = 10003
  receiver r{ctx, "0.0.0.0", "224.5.23.2", 10003};

  // 送信クラスの初期化
  // multicast_addr = 224.5.23.2, port = 10003
  sender s{ctx, "224.5.23.2", 10003};

  std::promise<std::uint64_t> promise{};
  r.on_status_updated([&promise](auto mps) { promise.set_value(mps); });

  // 別スレッドで受信を開始する
  auto t = run_io_context_in_new_thread(ctx);

  // 5回メッセージを送信する
  for (auto i = 0u; i < 5; ++i) {
    s.send("Hello!"s);
  }

  {
    promise = std::promise<std::uint64_t>{};

    // 最初の1秒間で受信したメッセージ数は5
    BOOST_TEST(promise.get_future().get() == 5);
  }

  {
    promise = std::promise<std::uint64_t>{};

    // 次の1秒間で受信したメッセージ数は0
    BOOST_TEST(promise.get_future().get() == 0);
  }
}

BOOST_AUTO_TEST_SUITE_END()
