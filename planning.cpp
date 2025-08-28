#include "planning.h"
#include <vector>
#include <queue>
#include <cmath>

struct Node {
    int x, y;
    double cost, priority;
    Node* parent;
    Node(int x, int y, double cost, double priority, Node* parent)
        : x(x), y(y), cost(cost), priority(priority), parent(parent) {}
};

std::vector<std::pair<int, int>> Planner::pathplanning(
    const std::vector<std::vector<int>>& grid,
    std::pair<int, int> start,
    std::pair<int, int> goal
) {
    int rows = grid.size(), cols = grid[0].size();
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    auto heuristic = [&](int x, int y) {
        return std::hypot(goal.first - x, goal.second - y);
    };

    std::priority_queue<
        std::pair<double, Node*>,
        std::vector<std::pair<double, Node*>>,
        std::greater<>
    > open;
    Node* startNode = new Node(start.first, start.second, 0, heuristic(start.first, start.second), nullptr);
    open.push({startNode->priority, startNode});

    Node* endNode = nullptr;
    while (!open.empty()) {
        Node* curr = open.top().second;
        open.pop();
        if (visited[curr->x][curr->y]) continue;
        visited[curr->x][curr->y] = true;
        if (curr->x == goal.first && curr->y == goal.second) {
            endNode = curr;
            break;
        }
        for (auto [dx, dy] : std::vector<std::pair<int, int>>{{0,1},{1,0},{0,-1},{-1,0}}) {
            int nx = curr->x + dx, ny = curr->y + dy;
            if (nx >= 0 && ny >= 0 && nx < rows && ny < cols && grid[nx][ny] == 0 && !visited[nx][ny]) {
                double newCost = curr->cost + 1;
                Node* next = new Node(nx, ny, newCost, newCost + heuristic(nx, ny), curr);
                open.push({next->priority, next});
            }
        }
    }
    std::vector<std::pair<int, int>> path;
    while (endNode) {
        path.push_back({endNode->x, endNode->y});
        endNode = endNode->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}