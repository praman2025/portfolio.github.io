#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <climits>

using namespace std;

struct Edge {
    int u, v, weight;
    bool operator<(const Edge& other) const {
        return weight < other.weight;
    }
};

class Graph {
public:
    int V;
    vector<vector<pair<int, int>>> adjList;

    Graph(int V) {
        this->V = V;
        adjList.resize(V);
    }

    void addEdge(int u, int v, int weight) {
        adjList[u].emplace_back(v, weight);
        adjList[v].emplace_back(u, weight);
    }

    vector<Edge> getAllEdges() {
        vector<Edge> edges;
        for (int u = 0; u < V; ++u) {
            for (auto& [v, weight] : adjList[u]) {
                if (u < v) {
                    edges.push_back({u, v, weight});
                }
            }
        }
        return edges;
    }
};

class DisjointSet {
public:
    vector<int> parent, rank;

    DisjointSet(int n) {
        parent.resize(n);
        rank.resize(n, 0);
        for (int i = 0; i < n; ++i) parent[i] = i;
    }

    int find(int x) {
        if (parent[x] != x) {
            parent[x] = find(parent[x]);
        }
        return parent[x];
    }

    void unite(int x, int y) {
        int rootX = find(x);
        int rootY = find(y);

        if (rootX != rootY) {
            if (rank[rootX] > rank[rootY]) {
                parent[rootY] = rootX;
            } else if (rank[rootX] < rank[rootY]) {
                parent[rootX] = rootY;
            } else {
                parent[rootY] = rootX;
                rank[rootX]++;
            }
        }
    }
};

vector<Edge> kruskalMST(Graph& graph) {
    vector<Edge> edges = graph.getAllEdges();
    sort(edges.begin(), edges.end());

    DisjointSet ds(graph.V);
    vector<Edge> mst;

    for (Edge& edge : edges) {
        if (ds.find(edge.u) != ds.find(edge.v)) {
            ds.unite(edge.u, edge.v);
            mst.push_back(edge);
        }
    }

    return mst;
}

vector<int> dijkstra(Graph& graph, int src) {
    vector<int> dist(graph.V, INT_MAX);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq;

    dist[src] = 0;
    pq.push({0, src});

    while (!pq.empty()) {
        auto [currentDist, u] = pq.top();
        pq.pop();

        if (currentDist > dist[u]) continue;

        for (auto& [v, weight] : graph.adjList[u]) {
            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                pq.push({dist[v], v});
            }
        }
    }

    return dist;
}

int main() {
    int V = 6; // Number of parking zones/nodes
    Graph graph(V);

    // Add edges (u, v, weight)
    graph.addEdge(0, 1, 4);
    graph.addEdge(0, 2, 3);
    graph.addEdge(1, 2, 1);
    graph.addEdge(1, 3, 2);
    graph.addEdge(2, 4, 5);
    graph.addEdge(3, 4, 1);
    graph.addEdge(3, 5, 3);
    graph.addEdge(4, 5, 2);

    // Hash map for parking availability
    unordered_map<int, int> parkingAvailability = {
        {0, 10}, // Zone 0 has 10 slots
        {1, 5},  // Zone 1 has 5 slots
        {2, 8},  // Zone 2 has 8 slots
        {3, 0},  // Zone 3 is full
        {4, 12}, // Zone 4 has 12 slots
        {5, 6}   // Zone 5 has 6 slots
    };

    // Find shortest path from source (0)
    int source = 0;
    vector<int> distances = dijkstra(graph, source);

    cout << "Shortest distances from source " << source << ":\n";
    for (int i = 0; i < V; ++i) {
        cout << "To zone " << i << " : " << distances[i] << "\n";
    }

    // Build MST using Kruskal's algorithm
    vector<Edge> mst = kruskalMST(graph);
    cout << "\nEdges in MST:\n";
    for (auto& edge : mst) {
        cout << "From " << edge.u << " to " << edge.v << " with weight " << edge.weight << "\n";
    }

    // Display parking availability
    cout << "\nParking Availability:\n";
    for (auto& [zone, slots] : parkingAvailability) {
        cout << "Zone " << zone << ": " << slots << " slots available\n";
    }

    return 0;
}
