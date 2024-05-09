#include <iostream>
#include <vector>
#include <queue>
#include <chrono>

using namespace std;
using namespace std::chrono;

vector<vector<pair<int, int>>> makeAdjList(int, const vector<vector<int>>&);
int findCheapestFlightWithDijkstra(int, const vector<vector<pair<int, int>>>&, int, int);
int findCheapestFlightWithBellmanFord(int, const vector<vector<pair<int, int>>>&, int, int);


int main() {
    vector<vector<int>> flights = {{0, 1, 120}, {1, 2, 150}, {2, 0, 110}, {1, 3, 600}, {2, 3, 200}};
    vector<vector<int>> flights2 = {{0, 1, 500}, {0, 2, 700}, {2, 1, -400}, {1, 3, 100}};


    vector<vector<pair<int, int>>> adjList = makeAdjList(4, flights);
    vector<vector<pair<int, int>>> adjList2 = makeAdjList(4, flights2);

    auto start = high_resolution_clock::now();
    int answerByDijkstra = findCheapestFlightWithDijkstra(4, adjList2, 0, 3);
    auto stop = high_resolution_clock::now();
    auto durationOfDijkstra = duration_cast<microseconds>(stop - start);

    start = high_resolution_clock::now();
    int answerByBellmanFord = findCheapestFlightWithBellmanFord(4, adjList2, 0, 3);
    stop = high_resolution_clock::now();
    auto durationOfBellmanFord = duration_cast<microseconds>(stop - start);

    cout << "Shortest cost by Dijkstra: " << answerByDijkstra << endl;
    cout << "Shortest cost by Bellman Ford: " << answerByBellmanFord << endl;
    cout << endl;
    cout << "Runtime of Dijkstra: " << durationOfDijkstra.count() << endl;
    cout << "Runtime of Bellman-Ford: " << durationOfBellmanFord.count() << endl;
}

/*
 * makeAdjList:
 * Converts vector<vector<int>> to an adjacency list vector<vector<pair<int, int>>>
 * Receives: n: Number of vertices (= airports) in the graph
 *           flights: Represents edges, {{Origin Apt, Dest Apt, Cost}, {Origin Apt, Dest Apt, Cost}, ... }
 * Returns: adjList: {Origin Apt: {Dest Apt, Cost}, {Dest Apt, Cost}, Origin Apt: {Dest Apt, Cost}... }
 */
vector<vector<pair<int, int>>> makeAdjList(int n, const vector<vector<int>>& flights) {
    vector<vector<pair<int, int>>> adjList(n);
    // (n) creates n empty vectors in adjList, without this, push_back() won't work.
    for (auto flight : flights) {
        adjList[flight[0]].push_back({flight[1], flight[2]});
    }
    return adjList;
}

/*
 * findCheapestFlightWithDijkstra:
 * Implements Dijkstra's algorithm, using priority queue to always pop the cheapest airport.
 * Receives : n: Number of vertices (= airports) in the graph
 *            adjList: Adjacency list, please refer to makeAdjList()
 *            src: Origin airport
 *            dst: Destination airport
 * Returns: costs[dst]: Cheapest cost from src to dst
 */
int findCheapestFlightWithDijkstra(int n, const vector<vector<pair<int, int>>>& adjList, int src, int dst) {
    vector<bool> visited(n, false); // We don't really need this boolean vector for a graph only with non-negative weights
    unordered_map<int, int> costs; // Stores the cheapest cost from src
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> minHeap;
    // Not for storage, but for always accessing the cheapest airport
    // {{Cost, Dest Apt}, {Cost, Dest Apt}, ... }

    for (int i = 0; i < n; i++) {
        if (i == src) costs.insert({i, 0});
        else costs.insert({i, INT_MAX});
    }

    minHeap.push({costs[src], src});

    while (!minHeap.empty()) {
        auto current = minHeap.top(); // {Cost, Dest Apt}
        int cost = current.first;
        int airport = current.second;
        visited[airport] = true;
        minHeap.pop();

        for (auto neighbor : adjList[airport]) { // neighbor: {Dest Apt, Cost}
            int neighborApt = neighbor.first;
            int flightCost = neighbor.second;
            if (!visited[neighborApt] && cost + flightCost < costs[neighborApt]) {
                costs[neighborApt] = cost + flightCost;
                minHeap.push({costs[neighborApt], neighborApt});
                // We only push airports connecting (directly and indirectly) to src airport
            }
        }
    }
    return costs[dst];
}

/*
 * findCheapestFlightWithBellmanFord:
 * Implements Bellman-Ford algorithm, simply using loop to iterate all airports (n-1) times.
 * Receives : n: Number of vertices (= airports) in the graph
 *            adjList: Adjacency list, please refer to makeAdjList()
 *            src: Origin airport
 *            dst: Destination airport
 * Returns: costs[dst]: Cheapest cost from src to dst
 */
int findCheapestFlightWithBellmanFord(int n, const vector<vector<pair<int, int>>>& adjList, int src, int dst) {
    unordered_map<int, int> costs;

    for (int i = 0; i < n; i++) {
        if (i == src) costs.insert({i, 0});
        else costs.insert({i, INT_MAX});
    }

    for (int i = 1; i < n; i++) { // Simply loops traversal of (n-1) times
        for (auto current: costs) {
            int airport = current.first;
            int cost = current.second;
            if (cost == INT_MAX) continue;
            // Need to skip otherwise cost + flightCost would be always overflow

            for (auto neighbor: adjList[airport]) {
                int neighborApt = neighbor.first;
                int flightCost = neighbor.second;
                if (cost + flightCost < costs[neighborApt]) {
                    costs[neighborApt] = cost + flightCost;
                }
            }
        }
    }
    return costs[dst];
}