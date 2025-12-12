/**
 * @file IGnssSource.hpp
 * @brief Abstract GNSS input interface for ATLAS.
 *
 * This file defines the minimal, vendor-independent GNSS interface used by ATLAS
 * to consume positioning data from professional GNSS receivers.
 *
 * The goal of this interface is to decouple the core SLAM and mapping logic from
 * any specific GNSS brand, protocol, or transport (serial, TCP, UDP, etc.).
 *
 * Concrete implementations may parse NMEA, RTCM, proprietary binary protocols,
 * or replay logged data for offline processing.
 */
#pragma once

#include <cstdint>
#include <optional>
#include <string>

/**
 * @namespace atlas::io
 * @brief Input/output interfaces for external sensors and data sources.
 */
namespace atlas::io {

/**
 * @enum GnssFixType
 * @brief Quality/type of the current GNSS position fix.
 */
enum class GnssFixType : uint8_t {
  None = 0, /**< No valid fix available */
  Single,   /**< Standalone GNSS fix */
  DGNSS,    /**< Differential GNSS fix */
  FloatRTK, /**< RTK float solution */
  FixedRTK, /**< RTK fixed (centimeter-level) solution */
  PPP       /**< Precise Point Positioning solution */
};

/**
 * @struct GnssFix
 * @brief Represents a single GNSS position fix.
 *
 * All fields are expressed in a sensor-agnostic form suitable for later
 * conversion into local or projected coordinate frames.
 */
struct GnssFix {
  /** Latitude in degrees (WGS84). */
  double lat_deg = 0.0;

  /** Longitude in degrees (WGS84). */
  double lon_deg = 0.0;

  /** Altitude above ellipsoid or mean sea level, depending on source (meters). */
  double alt_m = 0.0;

  /** GNSS fix quality/type. */
  GnssFixType type = GnssFixType::None;

  /** Horizontal dilution of precision (if available). */
  double hdop = 0.0;

  /** Number of satellites used in the solution. */
  uint8_t sats = 0;

  /**
   * Unix timestamp in milliseconds.
   *
   * This is optional and depends on whether the GNSS source provides
   * time information (e.g. via NMEA ZDA or proprietary messages).
   */
  uint64_t unix_time_ms = 0;
};

/**
 * @class IGnssSource
 * @brief Abstract interface for GNSS data sources.
 *
 * Implementations of this interface are responsible for:
 * - establishing communication with the GNSS device
 * - parsing incoming messages
 * - maintaining the most recent valid fix
 *
 * The interface is intentionally minimal to allow use in both real-time
 * and offline/replay scenarios.
 */
class IGnssSource {
 public:
  /** Virtual destructor for safe polymorphic deletion. */
  virtual ~IGnssSource() = default;

  /**
   * @brief Start GNSS data acquisition.
   *
   * This may spawn background threads, open serial/network connections,
   * or begin replaying logged data.
   *
   * @return true if the source started successfully, false otherwise.
   */
  virtual bool start() = 0;

  /**
   * @brief Stop GNSS data acquisition.
   *
   * Implementations should cleanly release resources and stop any
   * background processing.
   */
  virtual void stop() = 0;

  /**
   * @brief Retrieve the most recent GNSS fix.
   *
   * @return An optional containing the latest fix if available,
   *         or std::nullopt if no valid fix has been received yet.
   */
  virtual std::optional<GnssFix> latest_fix() const = 0;

  /**
   * @brief Human-readable name of the GNSS source.
   *
   * Useful for logging, debugging, and telemetry.
   */
  virtual std::string name() const = 0;
};

}  // namespace atlas::io