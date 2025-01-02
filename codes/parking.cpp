#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <climits>

using namespace std;

class MinHeap {
private:
    vector<int> heap;

    void heapifyUp(int index) {
        while (index > 0 && heap[parent(index)] > heap[index]) {
            swap(heap[parent(index)], heap[index]);
            index = parent(index);
        }
    }

    void heapifyDown(int index) {
        int smallest = index;
        if (left(index) < heap.size() && heap[left(index)] < heap[smallest])
            smallest = left(index);
        if (right(index) < heap.size() && heap[right(index)] < heap[smallest])
            smallest = right(index);
        if (smallest != index) {
            swap(heap[index], heap[smallest]);
            heapifyDown(smallest);
        }
    }

    int parent(int index) { return (index - 1) / 2; }
    int left(int index) { return 2 * index + 1; }
    int right(int index) { return 2 * index + 2; }

public:
    void insert(int value) {
        heap.push_back(value);
        heapifyUp(heap.size() - 1);
    }

    int extractMin() {
        if (heap.empty()) return -1;
        int root = heap[0];
        heap[0] = heap.back();
        heap.pop_back();
        heapifyDown(0);
        return root;
    }

    bool isEmpty() {
        return heap.empty();
    }
};

struct Edge {
    int src, dest, weight;
};

bool compareEdges(Edge a, Edge b) {
    return a.weight < b.weight;
}

class Graph {
public:
    int V;
    vector<vector<pair<int, int>>> adjList;

    Graph(int vertices) : V(vertices) {
        adjList.resize(vertices);
    }

    void addEdge(int u, int v, int weight) {
        adjList[u].push_back({v, weight});
        adjList[v].push_back({u, weight});
    }

    void dijkstra(int src) {
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

        for (int i = 0; i < V; ++i)
            cout << "Shortest distance to node " << i << " is " << dist[i] << endl;
    }

    int findParent(vector<int>& parent, int node) {
        if (parent[node] != node)
            parent[node] = findParent(parent, parent[node]);
        return parent[node];
    }

    void kruskal(vector<Edge>& edges) {
        sort(edges.begin(), edges.end(), compareEdges);
        vector<int> parent(V);
        for (int i = 0; i < V; ++i)
            parent[i] = i;
        vector<Edge> mst;

        for (auto edge : edges) {
            int srcParent = findParent(parent, edge.src);
            int destParent = findParent(parent, edge.dest);

            if (srcParent != destParent) {
                mst.push_back(edge);
                parent[srcParent] = destParent;
            }
        }

        for (auto edge : mst)
            cout << "Edge: " << edge.src << "-" << edge.dest << " Weight: " << edge.weight << endl;
    }
};

int main() {
    MinHeap parkingHeap;
    parkingHeap.insert(10);
    parkingHeap.insert(20);
    parkingHeap.insert(5);
    cout << "Nearest parking spot: " << parkingHeap.extractMin() << endl;

    Graph g(5);
    g.addEdge(0, 1, 10);
    g.addEdge(0, 4, 20);
    g.addEdge(1, 2, 30);
    g.addEdge(1, 3, 40);
    g.addEdge(3, 4, 50);

    g.dijkstra(0);

    vector<Edge> edges = {
        {0, 1, 10}, {0, 4, 20}, {1, 2, 30}, {1, 3, 40}, {3, 4, 50}
    };

    g.kruskal(edges);

    return 0;
}
