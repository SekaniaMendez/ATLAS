/**
 * @file NmeaSerialReader.hpp
 * @brief Utilities to discover GNSS-related serial ports (macOS/Linux) for ATLAS.
 *
 * This header intentionally keeps things simple: it provides helper functions to
 * list serial ports exposed by the operating system (e.g., macOS Bluetooth ports
 * under /dev/cu.*) and to heuristically identify ports that likely correspond to
 * GNSS receivers.
 *
 * Typical usage:
 * 1) Call list_all_ports() to get all candidate serial ports.
 * 2) Filter them with looks_like_gnss_port().
 * 3) If the filtered list is empty, show the full list to the user.
 *
 * @note These helpers do not open ports and do not parse NMEA. They are meant to
 *       support UI/selection and keep the GNSS reader/driver code clean.
 */

#ifndef ATLAS_GNSS_NMEA_SERIAL_READER_HPP
#define ATLAS_GNSS_NMEA_SERIAL_READER_HPP

#include <string>
#include <vector>

namespace atlas::gnss {

/**
 * @brief List all candidate serial ports exposed by the OS.
 *
 * On macOS this typically returns entries under /dev such as:
 * - /dev/cu.* (preferred for initiating outgoing connections)
 * - /dev/tty.*
 *
 * On Linux this may include devices such as /dev/ttyUSB*, /dev/ttyACM*, /dev/rfcomm*.
 *
 * @return A vector of absolute device paths.
 */
std::vector<std::string> list_all_ports();

/**
 * @brief Convert a string to lower-case (ASCII) for case-insensitive matching.
 *
 * @param s Input string.
 * @return Lower-cased string.
 */
std::string to_lower(std::string s);

/**
 * @brief Heuristic: determine whether a port name likely corresponds to a GNSS receiver.
 *
 * This is a best-effort check typically based on keywords in the port name
 * (e.g., "gnss", "gps", "chc", "i73", "rtk"), display in the UI.
 *
 * @param port_name Absolute device path (e.g., "/dev/cu.GNSS-3595478").
 * @return True if the name matches common GNSS-related keywords; false otherwise.
 */
bool looks_like_gnss_port(const std::string& port_name);

/**
 * @brief if no ports are found that could be gnss ports, print all the ports found
 */
void print_all_ports_if_no_gnss_ports_found();

}  // namespace atlas::gnss

#endif  // ATLAS_GNSS_NMEA_SERIAL_READER_HPP