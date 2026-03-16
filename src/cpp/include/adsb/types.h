#ifndef ADSB_TYPES_H
#define ADSB_TYPES_H

/**
 * ADS-B Aircraft Tracking Plugin Types
 * ======================================
 *
 * Consumes aircraft state vectors from:
 *   1. OpenSky Network REST API (free, rate-limited)
 *      GET https://opensky-network.org/api/states/all
 *   2. ADS-B Exchange (paid API)
 *   3. Raw Mode-S/ADS-B messages (1090 MHz)
 *
 * Output: $ADB FlatBuffer-aligned binary records
 *
 * Wire format:
 *   Header (16 bytes): magic[4]="$ADB", version(u32), source(u32), count(u32)
 *   N × AircraftRecord (128 bytes each, packed)
 *
 * Source enum: 0=OPENSKY, 1=ADSBX, 2=RAW_1090, 3=MLAT, 4=FLARM
 */

#include <cstdint>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>

namespace adsb {

// ============================================================================
// Constants
// ============================================================================

static constexpr char ADB_FILE_ID[4] = {'$', 'A', 'D', 'B'};
static constexpr uint32_t ADB_VERSION = 1;

// ============================================================================
// Enums
// ============================================================================

enum class DataSource : uint32_t {
    OPENSKY  = 0,
    ADSBX    = 1,
    RAW_1090 = 2,
    MLAT     = 3,
    FLARM    = 4,
};

enum class AircraftCategory : uint8_t {
    UNKNOWN         = 0,
    LIGHT           = 2,   // < 15500 lbs
    SMALL           = 3,   // 15500-75000 lbs
    LARGE           = 4,   // 75000-300000 lbs
    HIGH_VORTEX     = 5,   // e.g., B757
    HEAVY           = 6,   // > 300000 lbs
    HIGH_PERF       = 7,   // > 5g, > 400 kts
    ROTORCRAFT      = 8,
    GLIDER          = 9,
    LIGHTER_THAN_AIR = 10,
    PARACHUTIST     = 11,
    ULTRALIGHT      = 12,
    UAV             = 14,
    SPACE_VEHICLE   = 15,
    EMERGENCY_VEH   = 16,
    SERVICE_VEH     = 17,
};

enum class PositionSource : uint8_t {
    ADSB    = 0,
    ASTERIX = 1,
    MLAT    = 2,
    FLARM   = 3,
};

/// Special squawk codes
enum class SquawkAlert : uint8_t {
    NONE       = 0,
    EMERGENCY  = 1,  // 7700
    HIJACK     = 2,  // 7500
    COMMS_FAIL = 3,  // 7600
    MIL_INTERCEPT = 4, // 7777
};

// ============================================================================
// Core Record Types
// ============================================================================

#pragma pack(push, 1)

struct ADBHeader {
    char     magic[4];    // "$ADB"
    uint32_t version;
    uint32_t source;      // DataSource
    uint32_t count;
};
static_assert(sizeof(ADBHeader) == 16, "ADBHeader must be 16 bytes");

/// Aircraft state vector record (128 bytes packed)
struct AircraftRecord {
    // Identity
    char     icao24[8];       // ICAO 24-bit hex address (null-terminated)
    char     callsign[8];     // Flight callsign (null-terminated)
    char     origin_country[24]; // Country of registration

    // Time
    double   epoch_s;         // Unix timestamp of last contact [s]
    double   pos_epoch_s;     // Unix timestamp of last position update [s]

    // Position (WGS-84)
    double   lat_deg;         // Latitude [deg]
    double   lon_deg;         // Longitude [deg]
    float    baro_alt_m;      // Barometric altitude [m]
    float    geo_alt_m;       // Geometric altitude [m]

    // Velocity
    float    ground_speed_ms; // Ground speed [m/s]
    float    track_deg;       // True track (heading) [deg from N]
    float    vertical_rate_ms;// Vertical rate [m/s] (+up/-down)

    // Status
    uint8_t  on_ground;       // Boolean: on ground?
    uint8_t  spi;             // Special Purpose Indicator
    uint8_t  category;        // AircraftCategory enum
    uint8_t  pos_source;      // PositionSource enum

    // Transponder
    char     squawk[5];       // 4-digit squawk code (null-terminated)
    uint8_t  squawk_alert;    // SquawkAlert enum

    uint8_t  reserved[2];
};
static_assert(sizeof(AircraftRecord) == 104, "AircraftRecord size mismatch");

#pragma pack(pop)

// ============================================================================
// In-Memory Parsed State (from OpenSky JSON)
// ============================================================================

struct OpenSkyState {
    std::string icao24;
    std::string callsign;
    std::string origin_country;
    double      time_position = 0;
    double      last_contact = 0;
    double      longitude = NAN;
    double      latitude = NAN;
    double      baro_altitude = NAN;
    bool        on_ground = false;
    double      velocity = NAN;
    double      true_track = NAN;
    double      vertical_rate = NAN;
    double      geo_altitude = NAN;
    std::string squawk;
    bool        spi = false;
    int         position_source = 0;
    int         category = 0;
};

// ============================================================================
// Serialization
// ============================================================================

inline std::vector<uint8_t> serializeAircraft(const std::vector<AircraftRecord>& records,
                                                DataSource source = DataSource::OPENSKY) {
    size_t size = sizeof(ADBHeader) + records.size() * sizeof(AircraftRecord);
    std::vector<uint8_t> buf(size);

    ADBHeader hdr;
    std::memcpy(hdr.magic, ADB_FILE_ID, 4);
    hdr.version = ADB_VERSION;
    hdr.source = static_cast<uint32_t>(source);
    hdr.count = static_cast<uint32_t>(records.size());
    std::memcpy(buf.data(), &hdr, sizeof(ADBHeader));

    if (!records.empty()) {
        std::memcpy(buf.data() + sizeof(ADBHeader),
                    records.data(),
                    records.size() * sizeof(AircraftRecord));
    }
    return buf;
}

inline bool deserializeAircraft(const uint8_t* data, size_t len,
                                 ADBHeader& hdr,
                                 std::vector<AircraftRecord>& records) {
    if (len < sizeof(ADBHeader)) return false;
    std::memcpy(&hdr, data, sizeof(ADBHeader));
    if (std::memcmp(hdr.magic, ADB_FILE_ID, 4) != 0) return false;

    size_t expected = sizeof(ADBHeader) + hdr.count * sizeof(AircraftRecord);
    if (len < expected) return false;

    records.resize(hdr.count);
    if (hdr.count > 0) {
        std::memcpy(records.data(), data + sizeof(ADBHeader),
                    hdr.count * sizeof(AircraftRecord));
    }
    return true;
}

// ============================================================================
// Conversion: OpenSkyState → AircraftRecord
// ============================================================================

inline AircraftRecord openSkyToRecord(const OpenSkyState& os) {
    AircraftRecord r{};
    std::strncpy(r.icao24, os.icao24.c_str(), 7); r.icao24[7] = '\0';
    std::strncpy(r.callsign, os.callsign.c_str(), 7); r.callsign[7] = '\0';
    std::strncpy(r.origin_country, os.origin_country.c_str(), 23);
    r.origin_country[23] = '\0';

    r.epoch_s = os.last_contact;
    r.pos_epoch_s = os.time_position;
    r.lat_deg = os.latitude;
    r.lon_deg = os.longitude;
    r.baro_alt_m = std::isnan(os.baro_altitude) ? -1.0f : static_cast<float>(os.baro_altitude);
    r.geo_alt_m = std::isnan(os.geo_altitude) ? -1.0f : static_cast<float>(os.geo_altitude);
    r.ground_speed_ms = std::isnan(os.velocity) ? 0 : static_cast<float>(os.velocity);
    r.track_deg = std::isnan(os.true_track) ? 0 : static_cast<float>(os.true_track);
    r.vertical_rate_ms = std::isnan(os.vertical_rate) ? 0 : static_cast<float>(os.vertical_rate);
    r.on_ground = os.on_ground ? 1 : 0;
    r.spi = os.spi ? 1 : 0;
    r.category = static_cast<uint8_t>(os.category);
    r.pos_source = static_cast<uint8_t>(os.position_source);
    std::strncpy(r.squawk, os.squawk.c_str(), 4); r.squawk[4] = '\0';

    // Detect squawk alerts
    if (os.squawk == "7700") r.squawk_alert = static_cast<uint8_t>(SquawkAlert::EMERGENCY);
    else if (os.squawk == "7500") r.squawk_alert = static_cast<uint8_t>(SquawkAlert::HIJACK);
    else if (os.squawk == "7600") r.squawk_alert = static_cast<uint8_t>(SquawkAlert::COMMS_FAIL);
    else if (os.squawk == "7777") r.squawk_alert = static_cast<uint8_t>(SquawkAlert::MIL_INTERCEPT);
    else r.squawk_alert = static_cast<uint8_t>(SquawkAlert::NONE);

    return r;
}

// ============================================================================
// Filters
// ============================================================================

/// Filter by geographic bounding box
inline std::vector<AircraftRecord> filterByBBox(
    const std::vector<AircraftRecord>& records,
    double latMin, double latMax, double lonMin, double lonMax) {
    std::vector<AircraftRecord> out;
    for (const auto& r : records) {
        if (r.lat_deg >= latMin && r.lat_deg <= latMax &&
            r.lon_deg >= lonMin && r.lon_deg <= lonMax) {
            out.push_back(r);
        }
    }
    return out;
}

/// Filter by altitude range
inline std::vector<AircraftRecord> filterByAltitude(
    const std::vector<AircraftRecord>& records,
    float altMin_m, float altMax_m) {
    std::vector<AircraftRecord> out;
    for (const auto& r : records) {
        if (r.baro_alt_m >= altMin_m && r.baro_alt_m <= altMax_m) {
            out.push_back(r);
        }
    }
    return out;
}

/// Filter for emergency squawks only
inline std::vector<AircraftRecord> filterEmergencies(
    const std::vector<AircraftRecord>& records) {
    std::vector<AircraftRecord> out;
    for (const auto& r : records) {
        if (r.squawk_alert != static_cast<uint8_t>(SquawkAlert::NONE)) {
            out.push_back(r);
        }
    }
    return out;
}

/// Filter by aircraft category
inline std::vector<AircraftRecord> filterByCategory(
    const std::vector<AircraftRecord>& records, AircraftCategory cat) {
    std::vector<AircraftRecord> out;
    for (const auto& r : records) {
        if (r.category == static_cast<uint8_t>(cat)) out.push_back(r);
    }
    return out;
}

/// Detect potential military/intercept patterns
/// (High altitude, high speed, circling over area)
inline std::vector<AircraftRecord> detectHighPerf(
    const std::vector<AircraftRecord>& records,
    float minAlt_m = 12000, float minSpeed_ms = 250) {
    std::vector<AircraftRecord> out;
    for (const auto& r : records) {
        if (r.baro_alt_m >= minAlt_m && r.ground_speed_ms >= minSpeed_ms) {
            out.push_back(r);
        }
    }
    return out;
}

}  // namespace adsb

#endif  // ADSB_TYPES_H
