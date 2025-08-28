#include "odometry.h"
#include <vector>
#include <cmath>

std::vector<Odometry::Command> Odometry::computeCommands(
    const std::vector<std::pair<int, int>>& path,
    double wheelBase,
    double wheelRadius
) {
    std::vector<Command> commands;
    for (size_t i = 1; i < path.size(); ++i) {
        int dx = path[i].first - path[i-1].first;
        int dy = path[i].second - path[i-1].second;
        double angle = std::atan2(dy, dx);
        double distance = std::hypot(dx, dy);
        double time = distance / (wheelRadius * 2 * M_PI); // Simplified
        commands.push_back({angle, time});
    }
    return commands;
}