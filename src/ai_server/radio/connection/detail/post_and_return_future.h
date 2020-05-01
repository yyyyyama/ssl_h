#ifndef AI_SERVER_RADIO_CONNECTION_DETAIL_POST_AND_RETURN_FUTURE_H
#define AI_SERVER_RADIO_CONNECTION_DETAIL_POST_AND_RETURN_FUTURE_H

#include <future>
#include <type_traits>

#include <boost/asio/post.hpp>

namespace ai_server::radio::connection::detail {

template <class Executor, class Func>
inline auto post_and_return_future(Executor& ex, Func&& func)
    -> std::future<std::invoke_result_t<Func>> {
  std::promise<std::invoke_result_t<Func>> p{};
  auto f = p.get_future();
  boost::asio::post(ex, [func = std::forward<Func>(func), p = std::move(p)]() mutable {
    p.set_value(func());
  });
  return f;
}

} // namespace ai_server::radio::connection::detail

#endif // AI_SERVER_RADIO_CONNECTION_DETAIL_POST_AND_RETURN_FUTURE_H
