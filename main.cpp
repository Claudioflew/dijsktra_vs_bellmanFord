#include <iostream>
#include <vector>
#include <queue>

using namespace std;

vector<vector<pair<int, int>>> makeAdjList(int, vector<vector<int>>&);
int findCheapestFlightWithDijkstra(int, vector<vector<pair<int, int>>>&, int, int);
int findCheapestFlightWithBellmanFord(int, vector<vector<pair<int, int>>>&, int, int);


int main() {
    vector<vector<int>> flights = {{0, 1, 100}, {1, 2, 100}, {2, 0, 100}, {1, 3, 600}, {2, 3, 200}};

    vector<vector<pair<int, int>>> adjList = makeAdjList(4, flights);

    int answerByDijkstra = findCheapestFlightWithDijkstra(4, adjList, 0, 3);
    int answerByBellmanFord = findCheapestFlightWithBellmanFord(4, adjList, 0, 3);

    cout << "Shortest cost by Dijkstra: " << answerByDijkstra << endl;
    cout << "Shortest cost by Bellman Ford: " << answerByBellmanFord << endl;
}

vector<vector<pair<int, int>>> makeAdjList(int n, vector<vector<int>>& flights) {
    vector<vector<pair<int, int>>> adjList(n);
    for (auto flight : flights) {
        adjList[flight[0]].push_back({flight[1], flight[2]});
    }
    return adjList;
}

int findCheapestFlightWithDijkstra(int n, vector<vector<pair<int, int>>>& adjList, int src, int dst) {
    unordered_map<int, int> costs;
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> minHeap;

    for (int i = 0; i < n; i++) {
        if (i == src) costs.insert({i, 0});
        else costs.insert({i, INT_MAX});
    }

    minHeap.push({src, costs[src]});

    while (!minHeap.empty()) {
        auto current = minHeap.top();
        int airport = current.first;
        int cost = current.second;
        minHeap.pop();

        for (auto neighbor : adjList[airport]) {
            int neighborApt = neighbor.first;
            int flightCost = neighbor.second;
            if (cost + flightCost < costs[neighborApt]) {
                costs[neighborApt] = cost + flightCost;
                minHeap.push({neighborApt, costs[neighborApt]});
            }
        }
    }

    return costs[dst];
}

int findCheapestFlightWithBellmanFord(int n, vector<vector<pair<int, int>>>& adjList, int src, int dst) {
    unordered_map<int, int> costs;

    for (int i = 0; i < n; i++) {
        if (i == src) costs.insert({i, 0});
        else costs.insert({i, INT_MAX});
    }

    for (int i = 1; i < n; i++) {
        for (auto current: costs) {
            int airport = current.first;
            int cost = current.second;
            if (cost == INT_MAX) continue;

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