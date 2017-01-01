#include "serial.h"

namespace ai_server {
namespace util {

serial::serial(boost::asio::io_service& io_service, const std::string& device)
    : serial_(io_service, device) {
  serial_.set_option(boost::asio::serial_port::baud_rate(57600));
}

unsigned int serial::baud_rate() {
  boost::asio::serial_port::baud_rate b;
  serial_.get_option(b);
  return b.value();
}

unsigned int serial::character_size() {
  boost::asio::serial_port::character_size c;
  serial_.get_option(c);
  return c.value();
}

void serial::set_baud_rate(unsigned int value) {
  serial_.set_option(boost::asio::serial_port::baud_rate(value));
}

void serial::set_character_size(unsigned int value) {
  serial_.set_option(boost::asio::serial_port::character_size(value));
}

} // namespace util
} // namespace ai_server
