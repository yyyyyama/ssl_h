#include <string>
#include <fmt/format.h>

#include "detail/formatter.h"
#include "formatter.h"
#include "log_item.h"

using namespace fmt::literals;

namespace ai_server::logger {

std::string format(std::string_view fmt, const log_item& item) {
  // clang-format off
  return fmt::format(fmt.data()
                    , "elapsed"_a      = detail::elapsed{{item}}
                    , "level"_a        = detail::level{{item}}
                    , "level_simple"_a = detail::level_simple{{item}}
                    , "message"_a      = item.message
                    , "thread_id"_a    = detail::thread_id{{item}}
                    , "time"_a         = detail::time{{item}}
                    , "zone"_a         = item.zone_name
                    );
  // clang-format on
}

} // namespace ai_server::logger
