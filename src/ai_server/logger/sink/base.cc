#include "ai_server/logger/sink_registry.h"
#include "base.h"

namespace ai_server::logger::sink {

base::base() : base{{{"*", log_level::info}}} {}

base::base(levels_map_type levels)
    : levels_map_{std::move(levels)}, star_level_{levels_map_.find("*")} {
  sink_registry::global_sink_registry().register_sink(this);
}

base::base(base&& x) noexcept
    : levels_map_{std::move(x.levels_map_)}, star_level_{levels_map_.find("*")} {
  sink_registry::global_sink_registry().move(&x, this);
}

base::~base() noexcept {
  sink_registry::global_sink_registry().unregister_sink(this);
}

levels_map_type base::levels_map() const {
  return levels_map_;
}

void base::check_and_do_log(const log_item& item) {
  log_level level{};
  if (const auto r = levels_map_.find(item.zone_name.data()); r != levels_map_.cend()) {
    level = r->second;
  } else if (star_level_ != levels_map_.cend()) {
    level = star_level_->second;
  } else {
    return;
  }

  if (item.level >= level) {
    do_log(item);
  }
}

} // namespace ai_server::logger::sink
