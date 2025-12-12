/**
 * @file NmeaSerialReader.cpp
 * @brief Simple GNSS NMEA serial reader for ATLAS.
 *
 * This file implements a minimal, blocking serial reader used to validate
 * communication with GNSS receivers that output NMEA sentences.
 *
 * It is intentionally simple and is mainly intended for:
 * - early hardware bring-up
 * - debugging GNSS connectivity
 * - validating serial parameters
 *
 * This implementation is NOT a final GNSS driver. Production-ready GNSS
 * integration in ATLAS is expected to be implemented through the IGnssSource
 * interface.
 */

#include <asio.hpp>
#include <iostream>
#include <string>

/**
 * @brief Entry point for the NMEA serial reader.
 *
 * This program:
 * - opens a serial port using ASIO
 * - configures standard GNSS serial parameters
 * - blocks while reading line-delimited NMEA sentences
 * - prints each received sentence to stdout
 *
 * The program runs indefinitely until terminated by the user.
 *
 * @note This is a blocking implementation and does not perform checksum
 *       validation, message parsing, or threading.
 */
int main() {
  /**
   * ASIO I/O context used to manage all I/O operations.
   */
  asio::io_context io;

  /**
   * Serial device path.
   *
   * This must be updated to match the GNSS receiver's device path
   * (e.g. /dev/ttyUSB0 on Linux or /dev/tty.usbserial-* on macOS).
   */
  std::string port_name = "/dev/tty.usbserial-XXXX";

  /**
   * Serial baud rate.
   *
   * Common GNSS baud rates include 9600, 38400, and 115200 depending
   * on the receiver configuration.
   */
  unsigned int baud = 115200;

  /**
   * ASIO serial port object bound to the I/O context.
   */
  asio::serial_port port(io);

  /** Open the serial port device. */
  port.open(port_name);

  /** Configure serial port parameters. */
  port.set_option(asio::serial_port_base::baud_rate(baud));
  port.set_option(asio::serial_port_base::character_size(8));
  port.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
  port.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));

  /**
   * Stream buffer used to accumulate incoming serial data until
   * a full line is received.
   */
  asio::streambuf buf;

  /**
   * Main blocking read loop.
   *
   * Reads incoming data until a newline character is encountered,
   * extracts the line, and prints it to stdout.
   */
  while (true) {
    asio::read_until(port, buf, "\n");

    std::istream is(&buf);
    std::string line;
    std::getline(is, line);

    if (!line.empty()) {
      std::cout << line << "\n";
    }
  }
}