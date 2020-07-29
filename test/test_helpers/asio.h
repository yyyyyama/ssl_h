#ifndef AI_SERVER_TEST_TEST_HELPERS_ASIO_H
#define AI_SERVER_TEST_TEST_HELPERS_ASIO_H

#include <future>
#include <thread>

#include <boost/asio.hpp>

class run_io_context_in_new_thread {
  std::thread th_;
  boost::asio::io_context& ctx_;

public:
  run_io_context_in_new_thread(const run_io_context_in_new_thread&) = delete;
  run_io_context_in_new_thread(run_io_context_in_new_thread&&)      = delete;

  // io_context を別スレッドで run し、登録されたタスクが実行されるまでブロックする
  run_io_context_in_new_thread(boost::asio::io_context& ctx) : ctx_{ctx} {
    std::promise<void> p{};
    auto f = p.get_future();
    ctx_.post([&p] { p.set_value(); });
    th_ = std::thread{
        static_cast<std::size_t (boost::asio::io_context::*)()>(&boost::asio::io_context::run),
        &ctx_};
    f.get();
  }

  ~run_io_context_in_new_thread() {
    ctx_.stop();
    th_.join();
  }
};

#endif // AI_SERVER_TEST_ASIO_HELPER_H
