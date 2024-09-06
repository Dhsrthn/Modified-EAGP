#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <map>
#include <algorithm>
#include <random>

// Constants
const double PROPAGATION_DELAY = 0.00001;      // Propagation delay per meter
const double FRAME_TRANSMISSION_TIME = 0.0005; // Frame transmission time per frame (seconds)
const int FRAME_SIZE = 1024;                   // Frame size in bytes (for example)
const double ENERGY_PER_TRANSMISSION = 0.001;  // Energy consumed per transmission (arbitrary units)
const int NEIGHBORS_TO_GOSSIP = 3;             // Number of neighbors to gossip with at each step
const int NODES = 10;                          // Number of nodes in the network
const int MAX_TRANSMISSIONS = NODES;           // Maximum number of transmissions

class Node
{
public:
    double x, y;
    double range;
    int id;

    Node(double x, double y, double range, int id) : x(x), y(y), range(range), id(id) {}

    bool isWithinRange(const Node &other) const
    {
        double dist = std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
        return dist <= range;
    }
};

class Network
{
public:
    std::vector<Node> nodes;
    std::map<int, std::vector<int>> adjacencyList;
    double totalEnergy = 0.0;
    std::vector<std::vector<int>> allPaths;

    void addNode(double x, double y, double range, int id)
    {
        nodes.emplace_back(x, y, range, id);
        updateAdjacencyList();
    }

    void updateAdjacencyList()
    {
        adjacencyList.clear();
        for (const auto &node : nodes)
        {
            std::vector<int> neighbors;
            for (const auto &other : nodes)
            {
                if (node.id != other.id && node.isWithinRange(other))
                {
                    neighbors.push_back(other.id);
                }
            }
            adjacencyList[node.id] = neighbors;
        }
    }

    double calculateDistance(const Node &a, const Node &b) const
    {
        return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    }

    double calculateTransmissionTime(double distance) const
    {
        return distance * PROPAGATION_DELAY + FRAME_TRANSMISSION_TIME * FRAME_SIZE / 1024.0;
    }

    void printPath(const std::vector<int> &path) const
    {
        for (size_t i = 0; i < path.size(); ++i)
        {
            std::cout << path[i];
            if (i < path.size() - 1)
                std::cout << " -> ";
        }
        std::cout << std::endl;
    }

    void gossipRecursive(int currentId, int targetId, std::vector<int> path, int transmissionsLeft)
    {
        if (transmissionsLeft <= 0)
        {
            allPaths.push_back(path);
            return;
        }

        path.push_back(currentId);

        if (currentId == targetId)
        {
            allPaths.push_back(path);
            std::cout << "Gossiping path to target node " << targetId << ": ";
            printPath(path);
            path.pop_back();
            return;
        }

        std::vector<int> neighbors = adjacencyList[currentId];
        std::shuffle(neighbors.begin(), neighbors.end(), std::mt19937{std::random_device{}()});
        int count = std::min(static_cast<int>(neighbors.size()), NEIGHBORS_TO_GOSSIP);
        for (int i = 0; i < count; ++i)
        {
            int neighborId = neighbors[i];

            double distance = calculateDistance(nodes[currentId], nodes[neighborId]);
            double transmissionTime = calculateTransmissionTime(distance);
            totalEnergy += ENERGY_PER_TRANSMISSION;

            gossipRecursive(neighborId, targetId, path, transmissionsLeft - 1);
        }

        path.pop_back();
    }

    void startGossip(int sourceId, int targetId)
    {
        std::vector<int> path;
        gossipRecursive(sourceId, targetId, path, MAX_TRANSMISSIONS);
        std::cout << "Total energy consumed in gossiping: " << totalEnergy << " units" << std::endl;
        std::cout << "Total paths explored: " << allPaths.size() << std::endl;
    }
};

int main()
{
    std::srand(std::time(0));
    Network network;

    network.addNode(0, 0, 100, 1);
    network.addNode(20, 20, 150, 2);
    network.addNode(40, 40, 100, 3);
    network.addNode(60, 60, 150, 4);
    network.addNode(80, 80, 150, 5);
    network.addNode(100, 100, 150, 6);
    network.addNode(120, 120, 100, 7);
    network.addNode(140, 140, 150, 8);
    network.addNode(160, 160, 150, 9);
    network.addNode(180, 180, 150, 10);

    int sourceId = 1;
    int targetId = 10;

    std::cout << "Starting Gossip Protocol:" << std::endl;
    network.startGossip(sourceId, targetId);

    return 0;
}
