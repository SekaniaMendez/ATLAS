#include <asio.hpp>
#include <iostream>
#include <string>

int main() {
  asio::io_context io;

  std::string port_name = "/dev/tty.usbserial-XXXX";
  unsigned int baud = 115200;

  asio::serial_port port(io);
  port.open(port_name);
  port.set_option(asio::serial_port_base::baud_rate(baud));
  port.set_option(asio::serial_port_base::character_size(8));
  port.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
  port.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));

  asio::streambuf buf;

  while (true) {
    asio::read_until(port, buf, "\n");
    std::istream is(&buf);
    std::string line;
    std::getline(is, line);
    if (!line.empty()) std::cout << line << "\n";
  }
}