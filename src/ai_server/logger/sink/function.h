#ifndef AI_SERVER_LOGGER_SINK_FUNCTION_H
#define AI_SERVER_LOGGER_SINK_FUNCTION_H

#include <functional>
#include "base.h"

namespace ai_server::logger::sink {

/// ログ発生時に任意の関数を呼び出す sink
class function : public base {
  std::function<void(const log_item&)> f_;

public:
  template <class Func>
  function(Func&& f) : f_{std::forward<Func>(f)} {}

  template <class Func>
  function(levels_map_type levels, Func&& f)
      : base{std::move(levels)}, f_{std::forward<Func>(f)} {}

protected:
  void do_log(const log_item& item) override;
};

} // namespace ai_server::logger::sink

#endif // AI_SERVER_LOGGER_SINK_FUNCTION_H
