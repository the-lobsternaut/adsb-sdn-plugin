/**
 * ADS-B Raw Message Decoder — Implementation
 *
 * References:
 *   [1] ICAO Doc 9871: Technical Provisions for Mode S Services and
 *       Extended Squitter (2nd ed., 2012)
 *   [2] RTCA DO-260B: MOPS for 1090 MHz ADS-B (2009)
 *   [3] Junzi Sun, "The 1090 Megahertz Riddle" (2nd ed., TU Delft, 2021)
 *       https://mode-s.org/decode/
 *   [4] ICAO Annex 10 Vol IV: Surveillance Radar and Collision Avoidance
 *
 * CRC-24 polynomial: 0xFFF409 = x^24 + x^23 + x^10 + x^3 + 1
 * ADS-B Character set: #ABCDEFGHIJKLMNOPQRSTUVWXYZ##### ###############0123456789######
 */

#include "adsb/decoder.h"
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace adsb {

// ============================================================================
// Internal helper: modular floor (used by CPR decoding)
// ============================================================================

static int modFloor(int a, int b) {
    return ((a % b) + b) % b;
}

// ============================================================================
// ADS-B Character Lookup Table (6-bit to ASCII)
// Reference: [1] Table A-2-73A, [3] §4.1
// ============================================================================

static const char ADSB_CHARSET[64] = {
    '#', 'A', 'B', 'C', 'D', 'E', 'F', 'G',  // 0-7
    'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O',  // 8-15
    'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W',  // 16-23
    'X', 'Y', 'Z', '#', '#', '#', '#', '#',    // 24-31
    ' ', '#', '#', '#', '#', '#', '#', '#',     // 32-39
    '#', '#', '#', '#', '#', '#', '#', '#',     // 40-47
    '0', '1', '2', '3', '4', '5', '6', '7',    // 48-55
    '8', '9', '#', '#', '#', '#', '#', '#',     // 56-63
};

// ============================================================================
// Helper: Extract bits from byte array
// ============================================================================

/// Extract N bits starting at bit position 'start' (0-indexed from MSB of byte[0])
static uint32_t extractBits(const uint8_t* data, int start, int nbits) {
    uint32_t result = 0;
    for (int i = start; i < start + nbits; i++) {
        result = (result << 1) | ((data[i / 8] >> (7 - (i % 8))) & 1);
    }
    return result;
}

// ============================================================================
// CRC-24 — ICAO Annex 10 Vol IV §3.1.2.3.3
// ============================================================================

uint32_t crc24(const uint8_t* data, size_t bits) {
    uint32_t crc = 0;
    // Process each bit
    for (size_t i = 0; i < bits; i++) {
        uint8_t bit = (data[i / 8] >> (7 - (i % 8))) & 1;
        uint32_t invert = (crc >> 23) & 1;
        crc = ((crc << 1) | bit) & 0xFFFFFF;
        if (invert) {
            crc ^= CRC24_POLY;
        }
    }
    return crc;
}

bool validateCRC(const RawMessage& msg) {
    // For DF17/DF18 (112-bit): last 24 bits are PI field
    // CRC over all 112 bits should yield 0 for valid messages
    if (msg.length == 14) {
        uint32_t remainder = crc24(msg.bytes.data(), 112);
        return remainder == 0;
    }
    // For 56-bit messages: CRC over first 32 bits, XOR with last 24
    if (msg.length == 7) {
        uint32_t remainder = crc24(msg.bytes.data(), 56);
        return remainder == 0;
    }
    return false;
}

std::string extractICAO(const RawMessage& msg) {
    // ICAO address is in bytes 1-3 (bits 8-31) for DF17/DF18
    uint8_t df = getDF(msg);
    if (df == 17 || df == 18 || df == 11) {
        std::ostringstream oss;
        oss << std::hex << std::setfill('0')
            << std::setw(2) << (int)msg.bytes[1]
            << std::setw(2) << (int)msg.bytes[2]
            << std::setw(2) << (int)msg.bytes[3];
        return oss.str();
    }
    // For other DF: ICAO is XORed into the PI (CRC) field
    // We can recover it by computing CRC over data portion
    if (msg.length == 7) {
        uint32_t computed = crc24(msg.bytes.data(), 32);
        uint32_t pi = (static_cast<uint32_t>(msg.bytes[4]) << 16) |
                      (static_cast<uint32_t>(msg.bytes[5]) << 8) |
                       static_cast<uint32_t>(msg.bytes[6]);
        uint32_t icao = computed ^ pi;
        std::ostringstream oss;
        oss << std::hex << std::setfill('0') << std::setw(6) << icao;
        return oss.str();
    }
    return "";
}

// ============================================================================
// Downlink Format
// ============================================================================

uint8_t getDF(const RawMessage& msg) {
    // First 5 bits of byte 0
    return (msg.bytes[0] >> 3) & 0x1F;
}

uint8_t getTC(const RawMessage& msg) {
    // Type Code is bits 32-36 of the message (first 5 bits of ME field)
    // ME field starts at byte 4 for DF17
    return (msg.bytes[4] >> 3) & 0x1F;
}

// ============================================================================
// Aircraft Identification (TC 1-4) — [1] §A.2.3.1, [3] §4.1
// ============================================================================

std::optional<AircraftID> decodeIdentification(const RawMessage& msg) {
    if (msg.length != 14) return std::nullopt;
    uint8_t df = getDF(msg);
    if (df != 17 && df != 18) return std::nullopt;

    uint8_t tc = getTC(msg);
    if (tc < 1 || tc > 4) return std::nullopt;

    AircraftID id;
    id.icao24 = extractICAO(msg);
    id.type_code = tc;

    // Category (3 bits after TC): bits 37-39
    id.category = msg.bytes[4] & 0x07;

    // 8 characters × 6 bits each = 48 bits, starting at bit 40 (byte 5)
    // Bits 40-87
    uint64_t chars = 0;
    for (int i = 5; i < 11; i++) {
        chars = (chars << 8) | msg.bytes[i];
    }
    // We have 48 bits in the upper part of chars (bytes 5-10 = 48 bits)
    id.callsign.clear();
    for (int i = 7; i >= 0; i--) {
        uint8_t c = (chars >> (i * 6)) & 0x3F;
        char ch = ADSB_CHARSET[c];
        if (ch != '#') id.callsign += ch;
    }
    // Trim trailing spaces
    while (!id.callsign.empty() && id.callsign.back() == ' ') {
        id.callsign.pop_back();
    }

    return id;
}

// ============================================================================
// CPR Position — [2] §2.2.3.2.7.2, [3] §6
// ============================================================================

/// NL(lat) — Number of Longitude zones
/// Formula: NL = floor(2π / acos(1 - (1-cos(π/2/NZ)) / cos(π/180·|lat|)²))
/// where NZ = 15 (number of latitude zones for even message)
/// Reference: [2] Eq. 15, [3] §6.3
int cprNL(double lat) {
    if (std::abs(lat) >= 87.0) return 1;
    if (std::abs(lat) == 0.0) return 59;

    constexpr double NZ = 15.0;
    constexpr double PI = 3.14159265358979323846;
    double latRad = std::abs(lat) * PI / 180.0;
    double cosLat = std::cos(latRad);
    double numer = 1.0 - std::cos(PI / (2.0 * NZ));
    double denom = cosLat * cosLat;
    double inner = 1.0 - numer / denom;

    if (inner < -1.0) return 1;
    if (inner > 1.0) return 59;

    double nl = std::floor(2.0 * PI / std::acos(inner));
    return static_cast<int>(nl);
}

std::optional<CPRFrame> extractCPRFrame(const RawMessage& msg) {
    if (msg.length != 14) return std::nullopt;
    uint8_t df = getDF(msg);
    if (df != 17 && df != 18) return std::nullopt;

    uint8_t tc = getTC(msg);

    // Airborne position: TC 9-18 (baro alt) or TC 20-22 (GNSS alt)
    bool isBaro = (tc >= 9 && tc <= 18);
    bool isGNSS = (tc >= 20 && tc <= 22);
    if (!isBaro && !isGNSS) return std::nullopt;

    CPRFrame frame;
    frame.icao24 = extractICAO(msg);
    frame.type_code = tc;

    // ME field: bytes 4-10 (56 bits)
    // Surveillance status: bits 37-38
    // Single antenna flag: bit 39
    // Altitude: bits 40-51 (12 bits)
    // Time sync: bit 52 (T flag, ignored for airborne)
    // CPR format: bit 53 (F flag: 0=even, 1=odd)
    // Lat-CPR: bits 54-70 (17 bits)
    // Lon-CPR: bits 71-87 (17 bits)

    // Altitude (12 bits at position 40-51)
    uint16_t altCode = static_cast<uint16_t>(extractBits(msg.bytes.data(), 40, 12));
    bool qBit = (altCode >> 4) & 1;  // Q bit is bit 48 (8th from MSB of 12-bit field)
    frame.altitude_ft = static_cast<float>(decodeAltitude(altCode, qBit));

    // CPR odd/even flag (bit 53)
    frame.odd = (extractBits(msg.bytes.data(), 53, 1) == 1);

    // Lat-CPR (17 bits at position 54)
    frame.lat_cpr = extractBits(msg.bytes.data(), 54, 17);

    // Lon-CPR (17 bits at position 71)
    frame.lon_cpr = extractBits(msg.bytes.data(), 71, 17);

    frame.timestamp = msg.timestamp;

    return frame;
}

std::optional<DecodedPosition> decodeCPRGlobal(const CPRFrame& even, const CPRFrame& odd) {
    // Validate: must be even/odd pair from same aircraft
    if (even.odd || !odd.odd) return std::nullopt;
    if (even.icao24 != odd.icao24) return std::nullopt;

    constexpr double NZ = 15.0;
    constexpr double PI = 3.14159265358979323846;
    constexpr double CPR_MAX = 131072.0;  // 2^17

    // Scaled CPR values [0, 1)
    double latE = even.lat_cpr / CPR_MAX;
    double latO = odd.lat_cpr / CPR_MAX;
    double lonE = even.lon_cpr / CPR_MAX;
    double lonO = odd.lon_cpr / CPR_MAX;

    // Latitude zone sizes
    double dLatE = 360.0 / (4.0 * NZ);      // = 6.0 degrees
    double dLatO = 360.0 / (4.0 * NZ - 1.0); // ≈ 6.1017 degrees

    // Latitude zone index j
    int j = static_cast<int>(std::floor(59.0 * latE - 60.0 * latO + 0.5));

    // Compute latitude
    double latEven = dLatE * (modFloor(j, 60) + latE);
    double latOdd  = dLatO * (modFloor(j, 59) + latO);

    // Normalize to [-90, 90]
    if (latEven >= 270.0) latEven -= 360.0;
    if (latOdd >= 270.0) latOdd -= 360.0;

    // Check that both latitudes fall in the same NL zone
    if (cprNL(latEven) != cprNL(latOdd)) return std::nullopt;

    // Use the most recent frame to determine final latitude
    bool useOdd = (odd.timestamp >= even.timestamp);
    double lat = useOdd ? latOdd : latEven;

    // Longitude
    int nl = cprNL(lat);
    int ni = useOdd ? std::max(nl - 1, 1) : std::max(nl, 1);
    double dLon = 360.0 / ni;

    int m = static_cast<int>(std::floor(
        lonE * (cprNL(lat) - 1) - lonO * cprNL(lat) + 0.5));

    double lon;
    if (useOdd) {
        lon = dLon * (modFloor(m, ni) + lonO);
    } else {
        lon = dLon * (modFloor(m, ni) + lonE);
    }
    if (lon >= 180.0) lon -= 360.0;

    DecodedPosition pos;
    pos.icao24 = even.icao24;
    pos.lat_deg = lat;
    pos.lon_deg = lon;

    // Use the altitude from the more recent frame
    const CPRFrame& recent = useOdd ? odd : even;
    pos.alt_m = recent.altitude_ft * 0.3048;
    pos.alt_is_gnss = (recent.type_code >= 20);

    return pos;
}

std::optional<DecodedPosition> decodeCPRLocal(const CPRFrame& frame,
                                                double refLat, double refLon) {
    constexpr double NZ = 15.0;
    constexpr double CPR_MAX = 131072.0;
    constexpr double PI = 3.14159265358979323846;

    double dLat = frame.odd ? 360.0 / (4.0 * NZ - 1.0) : 360.0 / (4.0 * NZ);

    int j = static_cast<int>(std::floor(refLat / dLat)) +
            static_cast<int>(std::floor(
                0.5 + (std::fmod(refLat, dLat) / dLat) - frame.lat_cpr / CPR_MAX));

    double lat = dLat * (j + frame.lat_cpr / CPR_MAX);

    int nl = cprNL(lat);
    int ni = frame.odd ? std::max(nl - 1, 1) : std::max(nl, 1);
    double dLon = 360.0 / ni;

    int m = static_cast<int>(std::floor(refLon / dLon)) +
            static_cast<int>(std::floor(
                0.5 + (std::fmod(refLon, dLon) / dLon) - frame.lon_cpr / CPR_MAX));

    double lon = dLon * (m + frame.lon_cpr / CPR_MAX);
    if (lon >= 180.0) lon -= 360.0;

    DecodedPosition pos;
    pos.icao24 = frame.icao24;
    pos.lat_deg = lat;
    pos.lon_deg = lon;
    pos.alt_m = frame.altitude_ft * 0.3048;
    pos.alt_is_gnss = (frame.type_code >= 20);

    return pos;
}

// ============================================================================
// Altitude Decoding — [4] §3.1.2.6.5.4
// ============================================================================

int decodeAltitude(uint16_t altCode, bool isQ) {
    if (altCode == 0) return -1;

    if (isQ) {
        // Q-bit set: 25-ft increments
        // Remove Q-bit (bit 4), merge remaining 11 bits
        uint16_t n = ((altCode & 0xFE0) >> 1) | (altCode & 0x00F);
        return static_cast<int>(n) * 25 - 1000;
    } else {
        // Gillham code (100-ft increments with Gray code)
        // Reference: [3] §5.2.2 — convert Gray to binary
        // Extract C-bits and A-bits from the altitude code
        // C1 A1 C2 A2 C4 A4 _ B1 D1 B2 D2 B4 D4
        // (Q bit removed, positions are standard Gillham)

        // For simplicity, decode most common case
        // Full Gillham decode following ICAO standard:
        uint16_t c1 = (altCode >> 11) & 1;
        uint16_t a1 = (altCode >> 10) & 1;
        uint16_t c2 = (altCode >> 9) & 1;
        uint16_t a2 = (altCode >> 8) & 1;
        uint16_t c4 = (altCode >> 7) & 1;
        uint16_t a4 = (altCode >> 6) & 1;
        // bit 5 is Q/M bit
        uint16_t b1 = (altCode >> 4) & 1;
        uint16_t d1 = (altCode >> 3) & 1;
        uint16_t b2 = (altCode >> 2) & 1;
        uint16_t d2 = (altCode >> 1) & 1;
        uint16_t b4 = (altCode >> 0) & 1;

        // Gray→Binary for 500ft group
        uint16_t grayD = (d1 << 2) | (d2 << 1) | d1;  // D-bits
        uint16_t grayA = (a1 << 2) | (a2 << 1) | a4;
        uint16_t grayB = (b1 << 2) | (b2 << 1) | b4;

        // Convert Gray code D1D2D4 to binary
        uint16_t d = grayD;
        d ^= (d >> 1);

        // Convert Gray code A1A2A4 to binary
        uint16_t a = grayA;
        a ^= (a >> 1);

        // Convert Gray code B1B2B4 to binary
        uint16_t b = grayB;
        b ^= (b >> 1);

        // Hundreds: D encodes 0-4 in 500ft groups
        // C-bits encode the 100ft sub-group
        uint16_t grayC = (c1 << 2) | (c2 << 1) | c4;
        uint16_t c = grayC;
        c ^= (c >> 1);

        int alt500 = (d * 5 + a) * 2 + b;  // 500-ft groups
        int alt100;
        if (alt500 % 2) {
            alt100 = (6 - c) * 100;  // Odd 500-ft group: count down
        } else {
            alt100 = c * 100;        // Even 500-ft group: count up
        }

        return alt500 * 500 + alt100 - 1300;
    }
}

// ============================================================================
// Velocity Decoding (TC 19) — [1] §A.2.3.5, [3] §8
// ============================================================================

std::optional<DecodedVelocity> decodeVelocity(const RawMessage& msg) {
    if (msg.length != 14) return std::nullopt;
    uint8_t df = getDF(msg);
    if (df != 17 && df != 18) return std::nullopt;

    uint8_t tc = getTC(msg);
    if (tc != 19) return std::nullopt;

    DecodedVelocity vel;
    vel.icao24 = extractICAO(msg);

    // Subtype: bits 37-39
    vel.subtype = msg.bytes[4] & 0x07;

    if (vel.subtype == 1 || vel.subtype == 2) {
        // Ground speed — subtype 1: subsonic, subtype 2: supersonic
        // EW direction: bit 45
        // EW velocity: bits 46-55 (10 bits)
        // NS direction: bit 56
        // NS velocity: bits 57-66 (10 bits)
        // Vertical rate source: bit 67
        // Vertical rate sign: bit 68
        // Vertical rate: bits 69-77 (9 bits)

        int ewSign = extractBits(msg.bytes.data(), 45, 1) ? -1 : 1;
        int ewVel = static_cast<int>(extractBits(msg.bytes.data(), 46, 10)) - 1;
        int nsSign = extractBits(msg.bytes.data(), 56, 1) ? -1 : 1;
        int nsVel = static_cast<int>(extractBits(msg.bytes.data(), 57, 10)) - 1;

        if (ewVel < 0 || nsVel < 0) return std::nullopt;  // velocity unavailable

        // Supersonic: multiply by 4
        int factor = (vel.subtype == 2) ? 4 : 1;
        double vEW = ewSign * ewVel * factor;  // knots
        double vNS = nsSign * nsVel * factor;   // knots

        vel.ground_speed_kts = std::sqrt(vEW * vEW + vNS * vNS);
        vel.track_deg = std::atan2(vEW, vNS) * 180.0 / M_PI;
        if (vel.track_deg < 0) vel.track_deg += 360.0;

        // Vertical rate
        vel.vert_src_baro = (extractBits(msg.bytes.data(), 67, 1) == 0);
        int vrSign = extractBits(msg.bytes.data(), 68, 1) ? -1 : 1;
        int vrRaw = static_cast<int>(extractBits(msg.bytes.data(), 69, 9)) - 1;
        vel.vertical_rate_fpm = (vrRaw < 0) ? 0 : vrSign * vrRaw * 64.0;

    } else if (vel.subtype == 3 || vel.subtype == 4) {
        // Airspeed — heading and airspeed directly
        // Heading status: bit 45
        // Heading: bits 46-55 (10 bits)
        // Airspeed type: bit 56
        // Airspeed: bits 57-66 (10 bits)

        bool headingAvail = extractBits(msg.bytes.data(), 45, 1);
        if (headingAvail) {
            double heading = extractBits(msg.bytes.data(), 46, 10) * 360.0 / 1024.0;
            vel.track_deg = heading;  // Note: this is magnetic heading, not track
        }

        int factor = (vel.subtype == 4) ? 4 : 1;
        int aspd = static_cast<int>(extractBits(msg.bytes.data(), 57, 10)) - 1;
        vel.ground_speed_kts = (aspd < 0) ? 0 : aspd * factor;

        // Vertical rate (same position as subtype 1/2)
        vel.vert_src_baro = (extractBits(msg.bytes.data(), 67, 1) == 0);
        int vrSign = extractBits(msg.bytes.data(), 68, 1) ? -1 : 1;
        int vrRaw = static_cast<int>(extractBits(msg.bytes.data(), 69, 9)) - 1;
        vel.vertical_rate_fpm = (vrRaw < 0) ? 0 : vrSign * vrRaw * 64.0;
    } else {
        return std::nullopt;
    }

    return vel;
}

// ============================================================================
// Hex Message Parsing
// ============================================================================

std::optional<RawMessage> parseHex(const std::string& hex) {
    std::string clean;
    for (char c : hex) {
        if ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F')) {
            clean += c;
        }
    }

    // Mode S: 14 hex chars (56 bits) or 28 hex chars (112 bits)
    if (clean.size() != 14 && clean.size() != 28) return std::nullopt;

    RawMessage msg{};
    msg.length = static_cast<uint8_t>(clean.size() / 2);
    msg.timestamp = 0;
    msg.signal_dbm = 0;

    for (size_t i = 0; i < clean.size(); i += 2) {
        uint8_t byte = 0;
        for (int j = 0; j < 2; j++) {
            char c = clean[i + j];
            uint8_t nibble;
            if (c >= '0' && c <= '9') nibble = c - '0';
            else if (c >= 'a' && c <= 'f') nibble = 10 + c - 'a';
            else nibble = 10 + c - 'A';
            byte = (byte << 4) | nibble;
        }
        msg.bytes[i / 2] = byte;
    }

    return msg;
}

std::optional<RawMessage> parseBeast(const uint8_t* data, size_t len) {
    // Beast binary format: <1A> <type> <6-byte timestamp> <1-byte signal> <data>
    // Type: '1' = Mode-AC (2 bytes), '2' = Mode-S short (7), '3' = Mode-S long (14)
    if (len < 9) return std::nullopt;
    if (data[0] != 0x1A) return std::nullopt;

    int dataLen;
    switch (data[1]) {
        case '2': dataLen = 7; break;
        case '3': dataLen = 14; break;
        default: return std::nullopt;
    }

    if (len < static_cast<size_t>(8 + dataLen)) return std::nullopt;

    RawMessage msg{};
    msg.length = static_cast<uint8_t>(dataLen);

    // 6-byte timestamp (48-bit, 12MHz clock)
    uint64_t ts = 0;
    for (int i = 0; i < 6; i++) {
        ts = (ts << 8) | data[2 + i];
    }
    msg.timestamp = static_cast<double>(ts) / 12e6;  // Convert to seconds

    msg.signal_dbm = static_cast<int>(data[8]) - 128;

    for (int i = 0; i < dataLen; i++) {
        msg.bytes[i] = data[9 + i];
    }

    return msg;
}

// ============================================================================
// Aircraft Category — [1] Table A-2-73B
// ============================================================================

AircraftCategory categoryFromTC(uint8_t typeCode, uint8_t catNum) {
    // TC=1 is reserved (no category), TC=2-4 have subcategories
    if (typeCode == 4) {  // Category set A
        switch (catNum) {
            case 0: return AircraftCategory::UNKNOWN;
            case 1: return AircraftCategory::LIGHT;
            case 2: return AircraftCategory::SMALL;
            case 3: return AircraftCategory::LARGE;
            case 4: return AircraftCategory::HIGH_VORTEX;
            case 5: return AircraftCategory::HEAVY;
            case 6: return AircraftCategory::HIGH_PERF;
            case 7: return AircraftCategory::ROTORCRAFT;
            default: return AircraftCategory::UNKNOWN;
        }
    }
    if (typeCode == 3) {  // Category set B
        switch (catNum) {
            case 1: return AircraftCategory::GLIDER;
            case 2: return AircraftCategory::LIGHTER_THAN_AIR;
            case 3: return AircraftCategory::PARACHUTIST;
            case 4: return AircraftCategory::ULTRALIGHT;
            case 6: return AircraftCategory::UAV;
            case 7: return AircraftCategory::SPACE_VEHICLE;
            default: return AircraftCategory::UNKNOWN;
        }
    }
    if (typeCode == 2) {  // Category set C
        switch (catNum) {
            case 1: return AircraftCategory::EMERGENCY_VEH;
            case 3: return AircraftCategory::SERVICE_VEH;
            default: return AircraftCategory::UNKNOWN;
        }
    }
    return AircraftCategory::UNKNOWN;
}

std::string categoryDescription(AircraftCategory cat) {
    switch (cat) {
        case AircraftCategory::UNKNOWN:         return "Unknown";
        case AircraftCategory::LIGHT:           return "Light (< 15500 lbs)";
        case AircraftCategory::SMALL:           return "Small (15500-75000 lbs)";
        case AircraftCategory::LARGE:           return "Large (75000-300000 lbs)";
        case AircraftCategory::HIGH_VORTEX:     return "High vortex large (e.g., B757)";
        case AircraftCategory::HEAVY:           return "Heavy (> 300000 lbs)";
        case AircraftCategory::HIGH_PERF:       return "High performance (> 5g, > 400 kts)";
        case AircraftCategory::ROTORCRAFT:      return "Rotorcraft";
        case AircraftCategory::GLIDER:          return "Glider / sailplane";
        case AircraftCategory::LIGHTER_THAN_AIR:return "Lighter than air";
        case AircraftCategory::PARACHUTIST:     return "Parachutist / skydiver";
        case AircraftCategory::ULTRALIGHT:      return "Ultralight / hang-glider";
        case AircraftCategory::UAV:             return "Unmanned aerial vehicle";
        case AircraftCategory::SPACE_VEHICLE:   return "Space / transatmospheric vehicle";
        case AircraftCategory::EMERGENCY_VEH:   return "Emergency surface vehicle";
        case AircraftCategory::SERVICE_VEH:     return "Service surface vehicle";
        default:                                return "Unknown";
    }
}

}  // namespace adsb
