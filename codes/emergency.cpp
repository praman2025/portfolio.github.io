#include <iostream>
#include <vector>
#include <queue>
#include <climits>

using namespace std;

class Graph {
public:
    int V;
    vector<vector<pair<int, int>>> adjList;

    Graph(int vertices) : V(vertices) {
        adjList.resize(vertices);
    }

    void addEdge(int u, int v, int weight) {
        adjList[u].push_back({v, weight});
    }

    void dijkstra(int src, int hospital) {
        vector<int> dist(V, INT_MAX);
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
        pq.push({0, src});
        dist[src] = 0;

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            for (auto neighbor : adjList[u]) {
                int v = neighbor.first;
                int weight = neighbor.second;

                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    pq.push({dist[v], v});
                }
            }
        }

        cout << "Shortest distance to the nearest hospital (node " << hospital << ") is " << dist[hospital] << endl;
    }

    void floydWarshall(int src, int hospital) {
        vector<vector<int>> dist(V, vector<int>(V, INT_MAX));

        for (int i = 0; i < V; ++i)
            dist[i][i] = 0;

        for (int u = 0; u < V; ++u) {
            for (auto neighbor : adjList[u]) {
                int v = neighbor.first;
                int weight = neighbor.second;
                dist[u][v] = weight;
            }
        }

        for (int k = 0; k < V; ++k) {
            for (int i = 0; i < V; ++i) {
                for (int j = 0; j < V; ++j) {
                    if (dist[i][k] != INT_MAX && dist[k][j] != INT_MAX && dist[i][k] + dist[k][j] < dist[i][j]) {
                        dist[i][j] = dist[i][k] + dist[k][j];
                    }
                }
            }
        }

        cout << "Shortest distance to the nearest hospital (node " << hospital << ") is " << dist[src][hospital] << endl;
    }

    void bellmanFord(int src, int hospital) {
        vector<int> dist(V, INT_MAX);
        dist[src] = 0;

        for (int i = 1; i < V; ++i) {
            for (int u = 0; u < V; ++u) {
                for (auto neighbor : adjList[u]) {
                    int v = neighbor.first;
                    int weight = neighbor.second;

                    if (dist[u] != INT_MAX && dist[u] + weight < dist[v]) {
                        dist[v] = dist[u] + weight;
                    }
                }
            }
        }

        for (int u = 0; u < V; ++u) {
            for (auto neighbor : adjList[u]) {
                int v = neighbor.first;
                int weight = neighbor.second;

                if (dist[u] != INT_MAX && dist[u] + weight < dist[v]) {
                    cout << "Graph contains a negative weight cycle" << endl;
                    return;
                }
            }
        }

        cout << "Shortest distance to the nearest hospital (node " << hospital << ") is " << dist[hospital] << endl;
    }
};

int main() {
    Graph g(6);

    g.addEdge(0, 1, 2);
    g.addEdge(0, 2, 4);
    g.addEdge(1, 2, 1);
    g.addEdge(1, 3, 7);
    g.addEdge(2, 4, 3);
    g.addEdge(3, 4, 2);
    g.addEdge(3, 5, 1);
    g.addEdge(4, 5, 5);

    int start = 0;
    int hospital = 5;

    cout << "Using Dijkstra's Algorithm:" << endl;
    g.dijkstra(start, hospital);

    cout << "\nUsing Floyd-Warshall Algorithm:" << endl;
    g.floydWarshall(start, hospital);

    cout << "\nUsing Bellman-Ford Algorithm:" << endl;
    g.bellmanFord(start, hospital);

    return 0;
}
