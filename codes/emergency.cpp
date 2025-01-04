#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <climits>
#include <string>

// Dijkstra's Algorithm - Finding shortest path
void dijkstra(int graph[5][5], int V, int src) {
    std::vector<int> dist(V, INT_MAX);
    std::vector<bool> visited(V, false);
    dist[src] = 0;

    for (int count = 0; count < V - 1; count++) {
        int min = INT_MAX, u = -1;
        for (int v = 0; v < V; v++) {
            if (!visited[v] && dist[v] < min) {
                min = dist[v];
                u = v;
            }
        }

        visited[u] = true;
        for (int v = 0; v < V; v++) {
            if (!visited[v] && graph[u][v] && dist[u] != INT_MAX && dist[u] + graph[u][v] < dist[v]) {
                dist[v] = dist[u] + graph[u][v];
            }
        }
    }

    std::cout << "Shortest Path from Source (Node " << src << "):\n";
    for (int i = 0; i < V; i++) {
        std::cout << "Node " << i << ": " << dist[i] << " units\n";
    }
}

// BFS Algorithm - Searching through a graph
void bfs(int graph[5][5], int V, int start) {
    std::vector<bool> visited(V, false);
    std::queue<int> q;
    visited[start] = true;
    q.push(start);

    std::cout << "BFS Traversal from Node " << start << ":\n";
    while (!q.empty()) {
        int node = q.front();
        q.pop();
        std::cout << node << " ";

        for (int i = 0; i < V; i++) {
            if (graph[node][i] && !visited[i]) {
                visited[i] = true;
                q.push(i);
            }
        }
    }
    std::cout << "\n";
}

// Floyd-Warshall Algorithm - Finding shortest paths between all pairs
void floydWarshall(int graph[5][5], int V) {
    int dist[5][5];
    for (int i = 0; i < V; i++) {
        for (int j = 0; j < V; j++) {
            dist[i][j] = graph[i][j];
        }
    }

    for (int k = 0; k < V; k++) {
        for (int i = 0; i < V; i++) {
            for (int j = 0; j < V; j++) {
                if (dist[i][j] > dist[i][k] + dist[k][j]) {
                    dist[i][j] = dist[i][k] + dist[k][j];
                }
            }
        }
    }

    std::cout << "Shortest Path Matrix (Floyd-Warshall):\n";
    for (int i = 0; i < V; i++) {
        for (int j = 0; j < V; j++) {
            std::cout << dist[i][j] << " ";
        }
        std::cout << "\n";
    }
}

// Queue - Task scheduling in emergency response system
void useQueue() {
    std::queue<std::string> emergencyQueue;
    emergencyQueue.push("Fire Alarm in Zone A");
    emergencyQueue.push("Medical Emergency in Zone B");
    emergencyQueue.push("Traffic Accident in Zone C");

    std::cout << "Emergency Task Queue:\n";
    while (!emergencyQueue.empty()) {
        std::cout << emergencyQueue.front() << std::endl;
        emergencyQueue.pop();
    }
}

// Hashing - Storing and retrieving emergency event data
void useHashing() {
    std::unordered_map<int, std::string> emergencyMap;
    emergencyMap[101] = "Fire in Zone A";
    emergencyMap[102] = "Medical Emergency in Zone B";
    emergencyMap[103] = "Traffic Accident in Zone C";

    std::cout << "Emergency Event Lookup (Using Hashing):\n";
    std::cout << "Event 102: " << emergencyMap[102] << std::endl;
}

// Linked List - Storing dynamic emergency events
class EventNode {
public:
    std::string event;
    EventNode* next;
    EventNode(std::string event) : event(event), next(nullptr) {}
};

void useLinkedList() {
    EventNode* head = new EventNode("Fire in Zone A");
    head->next = new EventNode("Medical Emergency in Zone B");
    head->next->next = new EventNode("Traffic Accident in Zone C");

    std::cout << "Emergency Events (Using Linked List):\n";
    EventNode* current = head;
    while (current != nullptr) {
        std::cout << current->event << std::endl;
        current = current->next;
    }
}

int main() {
    std::cout << "Urban Emergency Response System\n\n";

    // Dijkstra's Algorithm for finding shortest path
    int graph[5][5] = {
        {0, 10, 0, 30, 0},
        {10, 0, 50, 0, 0},
        {0, 50, 0, 0, 0},
        {30, 0, 0, 0, 40},
        {0, 0, 0, 40, 0}
    };
    dijkstra(graph, 5, 0);

    // BFS Algorithm for traversing emergency zones
    bfs(graph, 5, 0);

    // Floyd-Warshall Algorithm for finding shortest paths between all zones
    floydWarshall(graph, 5);

    // Queue for managing emergency response tasks
    useQueue();

    // Hashing for quick lookup of emergency events
    useHashing();

    // Linked List for dynamically managing emergency events
    useLinkedList();

    return 0;
}
