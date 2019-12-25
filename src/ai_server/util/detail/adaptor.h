#ifndef AI_SERVER_UTIL_DETAIL_ADAPTOR_H
#define AI_SERVER_UTIL_DETAIL_ADAPTOR_H

#include <type_traits>
#include <boost/range/begin.hpp>
#include <boost/range/end.hpp>

namespace ai_server::util::detail {

// Boost.Range Adapterの自作方法
// https://www.boost.org/doc/libs/1_72_0/libs/range/doc/html/range/reference/extending/method_3/method_3_1.html

////////////////

// Boost.Rangeの要件を満たしているか
template <class T>
class is_range {
  template <class U>
  static auto check(typename boost::range_iterator<U>::type*) -> std::true_type;

  template <class U>
  static constexpr auto check(...) -> std::false_type;

public:
  static constexpr bool value = decltype(check<T>(nullptr))::value;
};

template <class T>
constexpr bool is_range_v = is_range<T>::value;

////////////////

// as_container, to_container用
// 参考元: https://github.com/faithandbrave/OvenToBoost/blob/master/boost/range/as_container.hpp
template <class Range, std::enable_if_t<is_range_v<Range>, std::nullptr_t> = nullptr>
class container_cast_wrapper {
  using range_iterator = typename boost::range_iterator<Range>::type;

  const Range& range_;

public:
  container_cast_wrapper(const Range& r) : range_(r) {}

  template <class Container,
            std::enable_if_t<std::is_constructible_v<Container, range_iterator, range_iterator>,
                             std::nullptr_t> = nullptr>
  operator Container() {
    return Container{boost::begin(range_), boost::end(range_)};
  }
};

// as_containerのタグ
struct as_conteiner_tag {};

// boost::rangeとas_containerを`|`で連結できるようにする
template <class Range>
inline auto operator|(Range&& r, as_conteiner_tag) {
  return container_cast_wrapper{std::forward<Range>(r)};
}

////////////////

} // namespace ai_server::util::detail

#endif
