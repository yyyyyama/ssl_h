#ifndef AI_SERVER_FILTER_VA_CALCULATOR_H
#define AI_SERVER_FILTER_VA_CALCULATOR_H

#include <chrono>

#include "base.h"

namespace ai_server {
namespace filter {

template <class T>
class va_calculator : public base<T, timing::on_updated> {
  using value_type      = T;
  using time_point_type = util::time_point_type;

public:
  va_calculator();

  value_type update(const value_type&, time_point_type) override;

private:
  time_point_type prev_time_;
  value_type prev_state_;
};

template <class T>
va_calculator<T>::va_calculator() : prev_time_(time_point_type::min()) {}

} // namespace filter
} // namespace ai_server

#endif // AI_SERVER_FILTER_VA_CALCULATOR_H
