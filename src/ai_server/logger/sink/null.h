#ifndef AI_SERVER_LOGGER_SINK_NULL_H
#define AI_SERVER_LOGGER_SINK_NULL_H

#include "base.h"

namespace ai_server::logger::sink {

/// 何もしない sink
class null final : public base {
public:
  using base::base;

protected:
  void do_log(const log_item& item) override;
};

} // namespace ai_server::logger::sink

#endif // AI_SERVER_LOGGER_SINK_NULL_H
