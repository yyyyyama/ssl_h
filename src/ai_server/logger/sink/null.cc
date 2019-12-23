#include "null.h"

namespace ai_server::logger::sink {

void null::do_log([[maybe_unused]] const log_item& item) {}

} // namespace ai_server::logger::sink
