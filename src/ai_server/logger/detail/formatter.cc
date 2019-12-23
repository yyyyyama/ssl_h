#include "formatter.h"

namespace fmt {

const std::chrono::steady_clock::time_point
    formatter<al::detail::elapsed>::elapsed_time_offset = std::chrono::steady_clock::now();

} // namespace fmt
