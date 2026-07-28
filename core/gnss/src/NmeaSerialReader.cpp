/**
 * @file NmeaSerialReader.hpp
 * @brief Simple GNSS NMEA serial reader for ATLAS.
 * This file implements a minimal, blocking serial reader used to validate
 * which ports could be gnss ports.
 */
#include <stdint.h>

#include <atlas/gnss/NmeaSerialReader.hpp>
#include <cctype>
#include <filesystem>

std::vector<std::string> list_all_ports() {
  namespace fs = std::filesystem;
  std::vector<std::string> ports;
  for (const auto& entry : fs::directory_iterator("/dev")) {
    const auto name = entry.path().filename().string();
    if (name.rfind("cu.", 0) == 0 || name.rfind("tty.", 0) == 0 || name.rfind("ttyUSB", 0) == 0 ||
        name.rfind("ttyACM", 0) == 0 || name.rfind("rfcomm", 0) == 0) {
      ports.push_back(entry.path().string());
    }
  }
  return ports;
}

static std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return (char)std::tolower(c); });
  return s;
}

bool looks_like_gnss_port(const std::string& port_name) {
  const auto lname = lower(port_name);
  return lname.find("gnss") != std::string::npos || lname.find("gps") != std::string::npos ||
         lname.find("chc") != std::string::npos || lname.find("i73") != std::string::npos ||
         lname.find("rtk") != std::string::npos;
}
