#include "serial.h"

namespace ai_server {
namespace util {

serial::serial(boost::asio::io_service& io_service, const std::string& device)
    : serial_(io_service, device) {
  serial_.set_option(boost::asio::serial_port::baud_rate(57600));
}

void serial::set_baud_rate(unsigned int value) {
  serial_.set_option(boost::asio::serial_port::baud_rate(value));
}

void serial::set_character_size(unsigned int value) {
  serial_.set_option(boost::asio::serial_port::character_size(value));
}

} // namespace util
} // namespace ai_server
