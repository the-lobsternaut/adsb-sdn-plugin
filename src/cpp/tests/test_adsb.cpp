/**
 * ADS-B Aircraft Tracking Plugin Tests
 */

#include "adsb/types.h"
#include <iostream>
#include <cassert>
#include <cstring>
#include <cmath>

using namespace adsb;

void testSerialization() {
    std::vector<AircraftRecord> records;

    // Simulated aircraft
    AircraftRecord r1{};
    std::strncpy(r1.icao24, "3c6444", 7);
    std::strncpy(r1.callsign, "DLH123", 7);
    std::strncpy(r1.origin_country, "Germany", 23);
    r1.epoch_s = 1710500000;
    r1.pos_epoch_s = 1710500000;
    r1.lat_deg = 50.1;
    r1.lon_deg = 8.7;
    r1.baro_alt_m = 11000;
    r1.geo_alt_m = 11050;
    r1.ground_speed_ms = 250;
    r1.track_deg = 90;
    r1.vertical_rate_ms = 0;
    r1.on_ground = 0;
    r1.category = static_cast<uint8_t>(AircraftCategory::LARGE);
    r1.pos_source = static_cast<uint8_t>(PositionSource::ADSB);
    std::strncpy(r1.squawk, "1000", 4);
    r1.squawk_alert = static_cast<uint8_t>(SquawkAlert::NONE);
    records.push_back(r1);

    // Emergency aircraft
    AircraftRecord r2{};
    std::strncpy(r2.icao24, "a1b2c3", 7);
    std::strncpy(r2.callsign, "UAL456", 7);
    std::strncpy(r2.origin_country, "United States", 23);
    r2.epoch_s = 1710500010;
    r2.lat_deg = 40.6;
    r2.lon_deg = -73.8;
    r2.baro_alt_m = 5000;
    r2.ground_speed_ms = 150;
    std::strncpy(r2.squawk, "7700", 4);
    r2.squawk_alert = static_cast<uint8_t>(SquawkAlert::EMERGENCY);
    records.push_back(r2);

    // UAV
    AircraftRecord r3{};
    std::strncpy(r3.icao24, "ff0001", 7);
    std::strncpy(r3.callsign, "DRONE01", 7);
    r3.lat_deg = 38.9;
    r3.lon_deg = -77.0;
    r3.baro_alt_m = 120;
    r3.ground_speed_ms = 15;
    r3.category = static_cast<uint8_t>(AircraftCategory::UAV);
    records.push_back(r3);

    // Serialize
    auto buf = serializeAircraft(records);
    assert(buf.size() == sizeof(ADBHeader) + 3 * sizeof(AircraftRecord));
    assert(std::memcmp(buf.data(), "$ADB", 4) == 0);

    // Deserialize
    ADBHeader hdr;
    std::vector<AircraftRecord> decoded;
    assert(deserializeAircraft(buf.data(), buf.size(), hdr, decoded));
    assert(decoded.size() == 3);
    assert(std::string(decoded[0].callsign) == "DLH123");
    assert(std::string(decoded[1].squawk) == "7700");

    std::cout << "  Serialization ✓ (" << buf.size() << " bytes, "
              << decoded.size() << " aircraft)\n";
}

void testFilters() {
    std::vector<AircraftRecord> records;

    auto make = [&](const char* icao, double lat, double lon, float alt,
                    float speed, uint8_t cat, const char* squawk) {
        AircraftRecord r{};
        std::strncpy(r.icao24, icao, 7);
        r.lat_deg = lat; r.lon_deg = lon;
        r.baro_alt_m = alt; r.ground_speed_ms = speed;
        r.category = cat;
        std::strncpy(r.squawk, squawk, 4);
        if (std::string(squawk) == "7700")
            r.squawk_alert = static_cast<uint8_t>(SquawkAlert::EMERGENCY);
        if (std::string(squawk) == "7500")
            r.squawk_alert = static_cast<uint8_t>(SquawkAlert::HIJACK);
        return r;
    };

    records.push_back(make("a1", 51.5, -0.1, 10000, 250, 4, "1000"));   // London, LARGE
    records.push_back(make("a2", 48.9, 2.3, 11000, 260, 6, "7700"));    // Paris, HEAVY, emergency
    records.push_back(make("a3", 40.6, -73.8, 300, 20, 14, "1200"));    // NYC, UAV, low alt
    records.push_back(make("a4", 35.0, 139.7, 12000, 300, 7, "4321"));  // Tokyo, HIGH_PERF
    records.push_back(make("a5", 55.7, 37.6, 13000, 280, 4, "7500"));   // Moscow, LARGE, hijack

    // BBox: Western Europe (lat 35-60, lon -10-40)
    auto europe = filterByBBox(records, 35, 60, -10, 40);
    assert(europe.size() == 3); // London + Paris + Moscow (37.6E is in bbox)

    // Altitude: > 10000m
    auto high = filterByAltitude(records, 10000, 99999);
    assert(high.size() == 4);

    // Emergencies
    auto emergencies = filterEmergencies(records);
    assert(emergencies.size() == 2); // Paris (7700) + Moscow (7500)

    // UAVs
    auto uavs = filterByCategory(records, AircraftCategory::UAV);
    assert(uavs.size() == 1);

    // High performance
    auto highPerf = detectHighPerf(records, 12000, 250);
    assert(highPerf.size() == 2); // Tokyo + Moscow

    std::cout << "  Filters ✓ (europe=" << europe.size()
              << " high=" << high.size()
              << " emergency=" << emergencies.size()
              << " uav=" << uavs.size()
              << " highPerf=" << highPerf.size() << ")\n";
}

void testOpenSkyConversion() {
    OpenSkyState os;
    os.icao24 = "3c6444";
    os.callsign = "DLH123";
    os.origin_country = "Germany";
    os.last_contact = 1710500000;
    os.time_position = 1710499995;
    os.latitude = 50.1;
    os.longitude = 8.7;
    os.baro_altitude = 11000;
    os.geo_altitude = 11050;
    os.velocity = 250;
    os.true_track = 90;
    os.vertical_rate = 0;
    os.on_ground = false;
    os.squawk = "7700";
    os.category = 4;

    auto r = openSkyToRecord(os);
    assert(std::string(r.icao24) == "3c6444");
    assert(std::string(r.callsign) == "DLH123");
    assert(std::abs(r.lat_deg - 50.1) < 0.01);
    assert(r.squawk_alert == static_cast<uint8_t>(SquawkAlert::EMERGENCY));

    std::cout << "  OpenSky conversion ✓\n";
}

void testWireFormat() {
    assert(sizeof(ADBHeader) == 16);
    assert(sizeof(AircraftRecord) == 104);

    auto empty = serializeAircraft({});
    assert(empty.size() == 16);

    ADBHeader hdr;
    std::vector<AircraftRecord> records;
    assert(deserializeAircraft(empty.data(), empty.size(), hdr, records));
    assert(hdr.count == 0);

    uint8_t bad[16] = {0};
    assert(!deserializeAircraft(bad, 16, hdr, records));

    std::cout << "  Wire format ✓ (header=16B, record=" << sizeof(AircraftRecord) << "B)\n";
}

int main() {
    std::cout << "=== adsb-sdn-plugin tests ===\n";
    testSerialization();
    testFilters();
    testOpenSkyConversion();
    testWireFormat();
    std::cout << "All ADS-B tests passed.\n";
    return 0;
}
