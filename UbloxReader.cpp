#include "UbloxReader.h"
#include <cstring>

bool UbloxReader::decodeUBX(const uint8_t* data, size_t length, double& lat, double& lon) {
    if (length < 92) return false; // UBX NAV-PVT payload is 92 bytes

    // Check fixType (offset 20): 3 = 3D fix, 4 = GNSS + dead reckoning
    uint8_t fixType = data[20];
    if (fixType < 3) return false;

    // Longitude (offset 24, int32_t, 1e-7 deg)
    int32_t rawLon;
    std::memcpy(&rawLon, data + 24, sizeof(int32_t));
    lon = rawLon * 1e-7;

    // Latitude (offset 28, int32_t, 1e-7 deg)
    int32_t rawLat;
    std::memcpy(&rawLat, data + 28, sizeof(int32_t));
    lat = rawLat * 1e-7;

    return true;
}