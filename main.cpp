#include "UbloxReader.h"
#include "planning.h"
#include "odometry.h"
#include <iostream>
#include <vector>
#include <utility>
#include <cstring>

int main() {
    // UBX decoding test (already present)
    UbloxReader reader;
    uint8_t data[92] = {0};
    data[20] = 3;
    int32_t lat_raw = static_cast<int32_t>(12.3456789 * 1e7);
    int32_t lon_raw = static_cast<int32_t>(98.7654321 * 1e7);
    std::memcpy(data + 28, &lat_raw, sizeof(int32_t));
    std::memcpy(data + 24, &lon_raw, sizeof(int32_t));
    double lat = 0, lon = 0;
    if (reader.decodeUBX(data, sizeof(data), lat, lon)) {
        std::cout << "Latitude: " << lat << "\nLongitude: " << lon << std::endl;
    } else {
        std::cout << "Failed to decode UBX data or fix not valid." << std::endl;
    }

    // Path planning test
    std::vector<std::vector<int>> grid = {
        {0,0,0,0,0},
        {0,1,1,1,0},
        {0,0,0,1,0},
        {0,1,0,0,0},
        {0,0,0,1,0}
    };
    Planner planner;
    auto path = planner.pathplanning(grid, {0,0}, {4,4});
    std::cout << "Path: ";
    for (auto& p : path) std::cout << "(" << p.first << "," << p.second << ") ";
    std::cout << std::endl;

    // Odometry test
    Odometry odo;
    auto commands = odo.computeCommands(path, 0.5, 0.1);
    std::cout << "Odometry Commands:\n";
    for (auto& cmd : commands)
        std::cout << "Angle: " << cmd.angle << ", Time: " << cmd.time << std::endl;

    return 0;
}