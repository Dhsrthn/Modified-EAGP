#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <ctime>
#include <map>
#include <thread>
#include <queue>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <algorithm>
#include <random>

// Constants
const double PROPAGATION_DELAY = 0.00001;      // Propagation delay per meter
const double FRAME_TRANSMISSION_TIME = 0.0005; // Frame transmission time per frame (seconds)
const int FRAME_SIZE = 1024;                   // Frame size in bytes (for example)
const double ENERGY_PER_TRANSMISSION = 0.001;  // Energy consumed per transmission (arbitrary units)
const int GOSSIP_FANOUT = 3;                   // Number of neighbors to gossip with
const double energyThresh = 0.005;             // Energy threshold to decide lazy/eager approach
const double energyLimit = 0.02;               // Energy limit after which the node stops functioning
const int TIMEOUT_MS = 1000;                   // Timeout period in milliseconds for lazy nodes
const int NODES = 10;                          // Number of nodes in the network
const int MAX_TRANSMISSIONS = NODES;           // Maximum number of transmissions

std::mutex mtx;
std::condition_variable cv;

class Node
{
public:
    double x, y;  
    double range; 
    int id;
    double energyUsed = 0.0; 

    std::queue<int> messageQueue; 
    bool isLazy = false;          

    Node(double x, double y, double range, int id) : x(x), y(y), range(range), id(id) {}

    bool isWithinRange(const Node &other) const
    {
        double dist = std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
        return dist <= range;
    }

    bool canFunction() const
    {
        return energyUsed <= energyLimit;
    }
};

// Network Class
class Network
{
public:
    std::vector<Node> nodes;
    std::map<int, std::vector<int>> adjacencyList;
    double totalEnergy = 0.0;     
    int pathsExplored = 0;        
    

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

    void transmitMessage(int currentId, int messageId, int targetId, std::vector<int> path)
    {
        if (nodes[currentId].energyUsed > energyThresh && nodes[currentId].energyUsed <= energyLimit)
        {
            gossipLazy(currentId, messageId, targetId, path);
        }
        else if (nodes[currentId].energyUsed <= energyThresh)
        {
            gossipEager(currentId, messageId, targetId, path);
        }
        else
        {
            std::cout << "Node " << currentId << " is out of energy and cannot function." << std::endl;
        }
    }

    void gossipEager(int currentId, int messageId, int targetId, std::vector<int> path)
    {
        path.push_back(currentId); 

        if (currentId == targetId)
        {
            std::cout << "Node " << currentId << " received message " << messageId << " successfully." << std::endl;
            std::cout << "Path taken: ";
            for (int nodeId : path)
            {
                std::cout << nodeId << " ";
            }
            std::cout << std::endl;

            std::lock_guard<std::mutex> lock(mtx);
            pathsExplored++;
            return;
        }

        std::cout << "Node " << currentId << " transmitting eagerly message " << messageId << std::endl;

        std::vector<int> neighbors = adjacencyList[currentId];
        std::shuffle(neighbors.begin(), neighbors.end(), std::mt19937{std::random_device{}()});
        int count = std::min(static_cast<int>(neighbors.size()), GOSSIP_FANOUT);

        for (int i = 0; i < count; ++i)
        {
            int neighborId = neighbors[i];

            double distance = calculateDistance(nodes[currentId], nodes[neighborId]);
            double transmissionTime = calculateTransmissionTime(distance);

            nodes[currentId].energyUsed += ENERGY_PER_TRANSMISSION;
            {
                std::lock_guard<std::mutex> lock(mtx);
                totalEnergy += ENERGY_PER_TRANSMISSION;
            }

            if (nodes[currentId].canFunction())
            {
                std::thread(&Network::transmitMessage, this, neighborId, messageId, targetId, path).detach();
            }
        }
    }

    void gossipLazy(int currentId, int messageId, int targetId, std::vector<int> path)
    {
        path.push_back(currentId); 

        if (currentId == targetId)
        {
            std::cout << "Node " << currentId << " received message " << messageId << " successfully." << std::endl;
            std::cout << "Path taken: ";
            for (int nodeId : path)
            {
                std::cout << nodeId << " ";
            }
            std::cout << std::endl;

            std::lock_guard<std::mutex> lock(mtx);
            pathsExplored++;
            return;
        }

        std::unique_lock<std::mutex> lock(mtx);

        std::cout << "Node " << currentId << " is lazy, waiting to transmit message " << messageId << std::endl;

        nodes[currentId].messageQueue.push(messageId);
        cv.wait_for(lock, std::chrono::milliseconds(TIMEOUT_MS));

        if (!nodes[currentId].messageQueue.empty() && nodes[currentId].messageQueue.front() == messageId)
        {
            nodes[currentId].messageQueue.pop();
            std::cout<<" Lasy Node "<<currentId<<" Transmitted message "<<std::endl;
            gossipEager(currentId, messageId, targetId, path); 
        }
        else
        {
            std::cout << "Node " << currentId << " dropped message " << messageId << " due to duplicate reception." << std::endl;
        }
    }

    void startEnergyAwareGossip(int sourceId, int messageId, int targetId)
    {
        std::vector<int> initialPath;
        transmitMessage(sourceId, messageId, targetId, initialPath);
    }

    void printSummary()
    {
        std::cout << "\nSummary:" << std::endl;
        std::cout << "Total energy used by all nodes EAGP: " << totalEnergy << std::endl;
        std::cout << "Number of paths explored EAGP: " << pathsExplored << std::endl;
    }

    void writeSummaryToFile(const std::string &filename)
    {
        std::ofstream outfile(filename);
        if (outfile.is_open())
        {
            outfile << "Summary:\n";
            outfile << "Total energy used by all nodes: " << totalEnergy << "\n";
            outfile << "Number of paths explored: " << pathsExplored << "\n";
            outfile.close();
            std::cout << "Summary written to " << filename << std::endl;
        }
        else
        {
            std::cerr << "Error opening file: " << filename << std::endl;
        }
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
    int messageId = 101; 
    int targetId = 10;

    std::cout << "Starting Energy-Aware Gossip Protocol:" << std::endl;
    network.startEnergyAwareGossip(sourceId, messageId, targetId);

    std::this_thread::sleep_for(std::chrono::seconds(5));

    network.printSummary();

    return 0;
}
