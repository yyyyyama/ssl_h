#ifndef AI_SERVER_UTIL_ADAPTOR_H
#define AI_SERVER_UTIL_ADAPTOR_H

#include "detail/adaptor.h"

namespace ai_server::util {

namespace {

/// @brief boost::rangeをコンテナに変換する
constexpr auto as_container = detail::as_conteiner_tag{};

} // namespace

/// @brief boost::rangeをコンテナに変換する
/// @param r 変換したいboost::rangeの値
/// @return コンテナに変換された値
template <class Range>
inline auto to_container(Range&& r) {
  return detail::container_cast_wrapper{std::forward<Range>(r)};
}

} // namespace ai_server::util

#endif
