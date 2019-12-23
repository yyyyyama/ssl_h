#include "function.h"

namespace ai_server::logger::sink {

void function::do_log(const log_item& item) {
  if (f_) f_(item);
}

} // namespace ai_server::logger::sink
