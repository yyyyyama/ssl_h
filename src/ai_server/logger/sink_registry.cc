#include "log_item.h"
#include "sink_registry.h"
#include "sink/base.h"

namespace ai_server::logger {

sink_registry& sink_registry::global_sink_registry() {
  static sink_registry sr{};
  return sr;
}

void sink_registry::register_sink(sink::base* sink) {
  std::lock_guard lock{mutex_};
  auto&& [_, done] = sinks_.emplace(sink);
  if (!done) {
    throw std::runtime_error("sink registered twice");
  }
}

void sink_registry::unregister_sink(sink::base* sink) noexcept {
  std::lock_guard lock{mutex_};
  sinks_.erase(sink);
}

void sink_registry::move(sink::base* from, sink::base* to) noexcept {
  if (from != to) {
    std::lock_guard lock{mutex_};
    sinks_.erase(from);
    sinks_.emplace(to);
  }
}

void sink_registry::notify_all(const log_item& item) {
  std::lock_guard lock{mutex_};
  for (auto&& s : sinks_) s->check_and_do_log(item);
}

} // namespace ai_server::logger
