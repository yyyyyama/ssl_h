#include "logger.h"

namespace ai_server::logger {

logger::logger(std::string zone_name) : zone_name_{std::move(zone_name)} {}

} // namespace ai_server::logger
