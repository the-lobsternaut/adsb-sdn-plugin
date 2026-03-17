#ifndef ADSB_DECODER_H
#define ADSB_DECODER_H

/**
 * ADS-B Raw Message Decoder
 * ==========================
 *
 * Decodes raw Mode S / ADS-B (1090ES) messages per:
 *   - ICAO Doc 9871 (Technical Provisions for Mode S)
 *   - RTCA DO-260B (ADS-B Airborne Equipment)
 *   - Junzi Sun, "The 1090 Megahertz Riddle" (2nd ed., 2021)
 *
 * Supports:
 *   - Downlink Format identification (DF0-24)
 *   - ICAO 24-bit address extraction
 *   - CPR (Compact Position Reporting) latitude/longitude decoding
 *   - Airborne velocity decoding (subtype 1-4)
 *   - Aircraft identification and category
 *   - CRC-24 validation (generator polynomial 0xFFF409)
 *
 * C++17, no external dependencies.
 */

#include "types.h"
#include <array>
#include <cstdint>
#include <cmath>
#include <string>
#include <optional>
#include <vector>

namespace adsb {

// ============================================================================
// Constants
// ============================================================================

/// CRC-24 generator polynomial for Mode S (x^24 + x^23 + x^10 + x^3 + 1)
static constexpr uint32_t CRC24_POLY = 0xFFF409;

/// Number of ADS-B frequency (MHz)
static constexpr double ADSB_FREQ_MHZ = 1090.0;

/// Downlink Format types
enum class DownlinkFormat : uint8_t {
    DF0  = 0,   // Short air-air surveillance (ACAS)
    DF4  = 4,   // Surveillance altitude reply
    DF5  = 5,   // Surveillance identity reply
    DF11 = 11,  // All-call reply
    DF16 = 16,  // Long air-air surveillance (ACAS)
    DF17 = 17,  // Extended squitter (ADS-B)
    DF18 = 18,  // Extended squitter (non-transponder)
    DF20 = 20,  // Comm-B altitude reply
    DF21 = 21,  // Comm-B identity reply
};

/// ADS-B Type Codes (TC) for DF17/DF18 messages
enum class TypeCode : uint8_t {
    AIRCRAFT_ID      = 1,   // TC 1-4: Aircraft identification
    SURFACE_POS      = 5,   // TC 5-8: Surface position
    AIRBORNE_POS_BA  = 9,   // TC 9-18: Airborne position (baro alt)
    AIRBORNE_VEL     = 19,  // TC 19: Airborne velocity
    AIRBORNE_POS_GA  = 20,  // TC 20-22: Airborne position (GNSS alt)
    RESERVED         = 23,  // TC 23-27: Reserved
    STATUS           = 28,  // TC 28: Aircraft status
    TARGET_STATE     = 29,  // TC 29: Target state and status
    OP_STATUS        = 31,  // TC 31: Aircraft operation status
};

// ============================================================================
// Decoded Message Structures
// ============================================================================

/// Decoded aircraft identification
struct AircraftID {
    std::string icao24;          // 24-bit ICAO address as hex string
    std::string callsign;        // 8-character flight ID
    uint8_t     category;        // Aircraft category (from TC and CA)
    uint8_t     type_code;       // Type code (1-4)
};

/// CPR encoded position (one frame — need even+odd pair to decode)
struct CPRFrame {
    std::string icao24;
    uint32_t    lat_cpr;         // 17-bit encoded latitude
    uint32_t    lon_cpr;         // 17-bit encoded longitude
    float       altitude_ft;     // Altitude in feet (-1 if unavailable)
    bool        odd;             // false=even (F=0), true=odd (F=1)
    double      timestamp;       // Reception time for surface position
    uint8_t     type_code;
};

/// Decoded position from CPR pair
struct DecodedPosition {
    std::string icao24;
    double      lat_deg;         // WGS-84 latitude
    double      lon_deg;         // WGS-84 longitude
    double      alt_m;           // Altitude in meters
    bool        alt_is_gnss;     // true if GNSS altitude, false if baro
};

/// Decoded velocity
struct DecodedVelocity {
    std::string icao24;
    double      ground_speed_kts;  // Ground speed [knots]
    double      track_deg;         // Track angle [degrees from N]
    double      vertical_rate_fpm; // Vertical rate [ft/min] (+ = climb)
    bool        vert_src_baro;     // true=baro, false=GNSS
    uint8_t     subtype;           // Velocity subtype (1-4)
};

/// Raw Mode S message (56 or 112 bits)
struct RawMessage {
    std::array<uint8_t, 14> bytes;  // Up to 14 bytes (112 bits)
    uint8_t     length;             // 7 (short) or 14 (long)
    double      timestamp;          // Reception timestamp
    int         signal_dbm;         // Signal strength (if available)
};

// ============================================================================
// CRC-24 Computation
// ============================================================================

/// Compute Mode S CRC-24 remainder
/// Reference: ICAO Annex 10 Vol IV, §3.1.2.3.3
uint32_t crc24(const uint8_t* data, size_t bits);

/// Validate a Mode S message CRC (returns true if valid)
bool validateCRC(const RawMessage& msg);

/// Extract ICAO address from a DF17/DF18 message
std::string extractICAO(const RawMessage& msg);

// ============================================================================
// Downlink Format Parsing
// ============================================================================

/// Get Downlink Format from first byte
uint8_t getDF(const RawMessage& msg);

/// Get Type Code from Extended Squitter ME field
uint8_t getTC(const RawMessage& msg);

// ============================================================================
// Aircraft Identification (TC 1-4)
// ============================================================================

/// Decode aircraft identification from DF17 message
/// Reference: ICAO Doc 9871, §A.2.3.1
std::optional<AircraftID> decodeIdentification(const RawMessage& msg);

// ============================================================================
// CPR Position Decoding
// ============================================================================

/// Extract CPR frame from airborne position message (TC 9-18, 20-22)
std::optional<CPRFrame> extractCPRFrame(const RawMessage& msg);

/// Decode globally unambiguous position from even/odd CPR pair
/// Reference: RTCA DO-260B, §2.2.3.2.7.2.6 (Global decoding)
/// The CPR algorithm uses NL(lat) zones per Eq. (15) in the spec.
std::optional<DecodedPosition> decodeCPRGlobal(const CPRFrame& even, const CPRFrame& odd);

/// Decode position using local reference (single CPR frame + known position)
/// Reference: RTCA DO-260B, §2.2.3.2.7.2.7 (Local decoding)
std::optional<DecodedPosition> decodeCPRLocal(const CPRFrame& frame,
                                                double refLat, double refLon);

/// Compute NL(lat) — number of longitude zones for given latitude
/// Reference: DO-260B Table 2-73 / "The 1090MHz Riddle" §6.3
int cprNL(double lat);

// ============================================================================
// Altitude Decoding
// ============================================================================

/// Decode Gillham (Mode C) altitude code to feet
/// Reference: ICAO Annex 10 Vol IV, §3.1.2.6.5.4
int decodeAltitude(uint16_t altCode, bool isQ);

// ============================================================================
// Velocity Decoding (TC 19)
// ============================================================================

/// Decode airborne velocity message
/// Reference: ICAO Doc 9871, §A.2.3.5
std::optional<DecodedVelocity> decodeVelocity(const RawMessage& msg);

// ============================================================================
// Hex Message Parsing
// ============================================================================

/// Parse hex string (e.g., from dump1090 "8D...") to RawMessage
std::optional<RawMessage> parseHex(const std::string& hex);

/// Parse Beast binary format frame
std::optional<RawMessage> parseBeast(const uint8_t* data, size_t len);

// ============================================================================
// Aircraft Type Lookup
// ============================================================================

/// Map ADS-B category to AircraftCategory enum
AircraftCategory categoryFromTC(uint8_t typeCode, uint8_t catNum);

/// Get human-readable description of aircraft category
std::string categoryDescription(AircraftCategory cat);

}  // namespace adsb

#endif  // ADSB_DECODER_H
