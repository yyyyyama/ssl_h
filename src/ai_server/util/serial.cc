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

serial::flow_control_t serial::flow_control() {
  boost::asio::serial_port::flow_control f;
  serial_.get_option(f);
  return f.value();
}

serial::parity_t serial::parity() {
  boost::asio::serial_port::parity p;
  serial_.get_option(p);
  return p.value();
}

serial::stop_bits_t serial::stop_bits() {
  boost::asio::serial_port::stop_bits s;
  serial_.get_option(s);
  return s.value();
}

void serial::set_baud_rate(unsigned int value) {
  serial_.set_option(boost::asio::serial_port::baud_rate(value));
}

void serial::set_character_size(unsigned int value) {
  serial_.set_option(boost::asio::serial_port::character_size(value));
}

void serial::set_flow_control(serial::flow_control_t value) {
  serial_.set_option(boost::asio::serial_port::flow_control(value));
}

void serial::set_parity(serial::parity_t value) {
  serial_.set_option(boost::asio::serial_port::parity(value));
}

void serial::set_stop_bits(serial::stop_bits_t value) {
  serial_.set_option(boost::asio::serial_port::stop_bits(value));
}

} // namespace util
} // namespace ai_server
