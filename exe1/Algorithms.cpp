#include <queue>
#include "Algorithms.hpp"
#include "Graph.hpp"
#include <unordered_map>
#include <stack>

#define INT_MAX 99999

namespace ariel
{

    bool Algorithms::isConnected(Graph g)
    {
        const vector<vector<int>> &adjMatrix = g.getAdjacencyMatrix();
        int numVertices = adjMatrix.size();

        // Initialize visited array to track visited vertices
        vector<bool> visited(numVertices, false);

        // Perform BFS traversal from the first vertex
        queue<int> q;
        q.push(0); // Start from vertex 0
        visited[0] = true;

        // BFS traversal
        while (!q.empty())
        {
            int curr = q.front();
            q.pop();
            for (int neighbor = 0; neighbor < numVertices; ++neighbor)
            {
                if (adjMatrix[curr][neighbor] && !visited[neighbor])
                {
                    q.push(neighbor);
                    visited[neighbor] = true;
                }
            }
        }

        // Check if all vertices are visited
        for (bool v : visited)
        {
            if (!v)
            {
                return false; // Graph is disconnected
            }
        }
        return true; // Graph is connected
    }

    string Algorithms::shortestPath(Graph g, int start, int end)
    {
        int numVertices = g.getNumVertices();
        const vector<vector<int>> &adjMatrix = g.getAdjacencyMatrix();

        queue<int> q;
        q.push(start);

        unordered_map<int, int> predecessors; // Map to store predecessors for path reconstruction
        vector<bool> visited(numVertices, false);
        visited[start] = true;

        // BFS traversal
        while (!q.empty())
        {
            int curr = q.front();
            q.pop();

            // If the destination vertex is reached, reconstruct and return the shortest path
            if (curr == end)
            {
                string path = to_string(curr);
                int pred = predecessors[curr];
                while (pred != start)
                {
                    path = to_string(pred) + " -> " + path;
                    pred = predecessors[pred];
                }
                path = to_string(start) + " -> " + path;
                return path;
            }

            // Enqueue unvisited neighbors of the current vertex
            for (int neighbor = 0; neighbor < numVertices; ++neighbor)
            {
                if (adjMatrix[curr][neighbor] && !visited[neighbor])
                {
                    q.push(neighbor);
                    visited[neighbor] = true;
                    predecessors[neighbor] = curr; // Record predecessor for path reconstruction
                }
            }
        }

        // If the destination vertex is not reachable
        return "No path exists from vertex " + to_string(start) + " to vertex " + to_string(end);
    }

void dfs(Graph g, int start, vector<bool> &visited) {
    // Mark the starting vertex as visited
    visited[start] = true;

    // Get the neighbors of the starting vertex
    const vector<vector<int>> &adjMatrix = g.getAdjacencyMatrix();
    for (size_t neighbor = 0; neighbor < adjMatrix[start].size(); ++neighbor) {
        // If the edge exists and the neighbor vertex is not visited, recursively call dfs
        if (adjMatrix[start][neighbor] != 0 && !visited[neighbor]) {
            dfs(g, neighbor, visited);
        }
    }
}

string dfsDetectCycle(const vector<vector<int>> &adjMatrix, size_t v, vector<bool> &visited, vector<bool> &isVisiting, int parent, vector<int> &path) {
    visited[v] = true;
    isVisiting[v] = true;
    path.push_back(v);

    // Traverse all neighbors of vertex v
    for (size_t neighbor = 0; neighbor < adjMatrix[v].size(); ++neighbor) {
        if (adjMatrix[v][neighbor] != 0) {
            if (!visited[neighbor]) {
                // If the neighbor is not visited, recursively call the function
                string cycle = dfsDetectCycle(adjMatrix, neighbor, visited, isVisiting, v, path);
                if (!cycle.empty()) {
                    // Cycle detected, return the cycle string
                    if (cycle == "-1") {
                        return to_string(v);
                    } else {
                        return to_string(v) + " -> " + cycle;
                    }
                }
            } else if (isVisiting[neighbor] && static_cast<int>(neighbor) != parent) {
                // If the neighbor is currently being visited and not the parent of v, there is a cycle
                // Reconstruct and return the cycle string
                string cycle;
                int idx = static_cast<int>(path.size()) - 1;
                while (idx >= 0 && path[idx] != static_cast<int>(neighbor)) {
                    cycle = to_string(path[idx]) + " -> " + cycle;
                    idx--;
                }
                cycle = to_string(neighbor) + " -> " + cycle + to_string(neighbor);
                return cycle;
            }
        }
    }

    // Backtrack: Remove the vertex from the current path
    path.pop_back();
    isVisiting[v] = false;

    return ""; // No cycle detected
}

/**
 * @brief Function to check whether the graph contains a cycle.
 * 
 * @param g The graph to check for cycles.
 * @return The cycle string if a cycle is detected, an empty string otherwise.
 */
string Algorithms::isContainsCycle(Graph g) {
    // Get the adjacency matrix of the graph
    const vector<vector<int>> &adjMatrix = g.getAdjacencyMatrix();
    size_t numVertices = adjMatrix.size();

    // Vector to keep track of visited vertices
    vector<bool> visited(numVertices, false);
    // Vector to keep track of vertices currently being visited in the DFS traversal
    vector<bool> isVisiting(numVertices, false);
    // Vector to store the current path in the DFS traversal
    vector<int> path;

    // Perform DFS traversal from each vertex
    for (size_t i = 0; i < numVertices; ++i) {
        if (!visited[i]) {
            // Check for cycle starting from vertex i
            string cycle = dfsDetectCycle(adjMatrix, i, visited, isVisiting, -1, path);
            if (!cycle.empty()) {
                return cycle;
            }
        }
    }

    return "0"; // No cycle found
}

    //    string Algorithms::isContainsCycle(Graph g) {
    //     int numVertices = g.getNumVertices();
    //     vector<bool> visited(numVertices, false); // Vector to track visited vertices

    //     // Perform DFS traversal from each vertex
    //     for (int i = 0; i < numVertices; ++i) {
    //         if (!visited[i]) {
    //             stack<int> s;
    //             s.push(i);
    //             vector<int> parent(numVertices, -1); // Vector to store parent of each vertex

    //             while (!s.empty()) {
    //                 int v = s.top();
    //                 s.pop();

    //                 if (!visited[v]) {
    //                     visited[v] = true;

    //                     for (int neighbor = numVertices - 1; neighbor >= 0; --neighbor) {
    //                         if (g.getAdjacencyMatrix()[v][neighbor]) {
    //                             if (!visited[neighbor]) {
    //                                 s.push(neighbor);
    //                                 parent[neighbor] = v; // Set parent of neighbor
    //                             } else if (parent[v] != neighbor) {
    //                                 // Back edge detected, construct cycle
    //                                 string cycle;
    //                                 int u = v;
    //                                 while (u != neighbor) {
    //                                     cycle = to_string(u) + " -> " + cycle; // Add parent vertex to cycle string
    //                                     u = parent[u]; // Trace back to parent vertices
    //                                 }
    //                                 cycle = to_string(neighbor) + " -> " + cycle + to_string(neighbor); // Add neighbor vertex again to close the cycle
    //                                 return "The cycle is: " + cycle;
    //                             }
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //     }
    //     // No back edges detected, so there's no cycles
    //     return "No cycle found";
    // }

    string Algorithms::isBipartite(Graph g)
    {
        int numVertices = g.getNumVertices();
        const vector<vector<int>> &adjMatrix = g.getAdjacencyMatrix();
        vector<int> colors(numVertices, -1); // Vector to store vertex colors (-1 for uncolored, 0 and 1 for two colors)
        queue<int> q;
        vector<int> partA, partB;
        bool isBipartite = true;

        // Perform BFS traversal from each vertex
        for (int i = 0; i < numVertices; ++i)
        {
            if (colors[i] == -1)
            {
                q.push(i);
                colors[i] = 0;      // Color vertex i with color 0
                partA.push_back(i); // Add vertex i to partition A

                while (!q.empty())
                {
                    int curr = q.front();
                    q.pop();

                    // Assign opposite color to neighbors
                    int nextColor = 1 - colors[curr]; // Toggle the color (0 to 1 or 1 to 0)
                    for (int neighbor = 0; neighbor < numVertices; ++neighbor)
                    {
                        if (adjMatrix[curr][neighbor])
                        {
                            // If neighbor is uncolored, assign the opposite color and enqueue it
                            if (colors[neighbor] == -1)
                            {
                                colors[neighbor] = nextColor;
                                q.push(neighbor);
                                // Add neighbor to the corresponding partition
                                if (nextColor == 0)
                                {
                                    partA.push_back(neighbor);
                                }
                                else
                                {
                                    partB.push_back(neighbor);
                                }
                            }
                            else if (colors[neighbor] != nextColor)
                            {
                                // If neighbor is already colored and has the same color as curr, the graph is not bipartite
                                isBipartite = false;
                                break;
                            }
                        }
                    }
                }
            }
        }

        // Construct the partition string
        string partition;
        if (isBipartite)
        {
            partition += "The graph is bipartite: A={";
            for (int vertex : partA)
            {
                partition += to_string(vertex) + ", ";
            }
            partition.pop_back(); // Remove trailing comma and space
            partition.pop_back();
            partition += "}, B={";
            for (int vertex : partB)
            {
                partition += to_string(vertex) + ", ";
            }
            partition.pop_back(); // Remove trailing comma and space
            partition.pop_back();
            partition += "}";
        }
        else
        {
            partition = "0";
        }

        return partition;
    }

    string Algorithms::negativeCycle(Graph g)
    {
        int numVertices = g.getNumVertices();
        const vector<vector<int>> &adjMatrix = g.getAdjacencyMatrix();

        // Initialize distance array
        vector<int> dist(numVertices, INT_MAX);
        dist[0] = 0; // Set distance from source vertex to itself as 0

        // Relax edges repeatedly to find shortest paths
        for (int i = 0; i < numVertices - 1; ++i)
        {
            for (int u = 0; u < numVertices; ++u)
            {
                for (int v = 0; v < numVertices; ++v)
                {
                    if (adjMatrix[u][v] && dist[u] != INT_MAX && dist[u] + adjMatrix[u][v] < dist[v])
                    {
                        dist[v] = dist[u] + adjMatrix[u][v];
                    }
                }
            }
        }

        // Check for negative cycles
        for (int u = 0; u < numVertices; ++u)
        {
            for (int v = 0; v < numVertices; ++v)
            {
                if (adjMatrix[u][v] && dist[u] != INT_MAX && dist[u] + adjMatrix[u][v] < dist[v])
                {
                    return "The graph contains a negative cycle.";
                }
            }
        }

        return "The graph does not contain a negative cycle.";
    }
}
