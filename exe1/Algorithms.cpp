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
        vector<bool> visited(static_cast<size_t>(numVertices), false);

        // Perform BFS traversal from the first vertex
        queue<size_t> q;
        q.push(0); // Start from vertex 0
        visited[0] = true;

        // BFS traversal
        while (!q.empty())
        {
            size_t curr = q.front();
            q.pop();
            for (size_t neighbor = 0; neighbor < numVertices; ++neighbor)
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
        size_t numVertices = g.getNumVertices();
        const vector<vector<int>> &adjMatrix = g.getAdjacencyMatrix();

        queue<size_t> q;
        q.push(static_cast<size_t>(start));

        unordered_map<int, int> predecessors; // Map to store predecessors for path reconstruction
        vector<bool> visited(numVertices, false);
        visited[static_cast<size_t>(start)] = true;

        // BFS traversal
        while (!q.empty())
        {
            size_t curr = q.front();
            q.pop();

            // If the destination vertex is reached, reconstruct and return the shortest path
            if (curr == end)
            {
                string path = to_string(curr);
                int pred = predecessors[curr];
                while (pred != start)
                {
                    path = to_string(pred) + "->" + path;
                    pred = predecessors[pred];
                }
                path = to_string(start) + "->" + path;
                return path;
            }

            // Enqueue unvisited neighbors of the current vertex
            for (size_t neighbor = 0; neighbor < numVertices; ++neighbor)
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
        return "-1";
    }

    bool Algorithms::isContainsCycle(Graph g)
    {
        size_t numVertices = g.getNumVertices();
        vector<bool> visited(numVertices, false); // Vector to track visited vertices

        // Perform DFS traversal from each vertex
        for (size_t i = 0; i < numVertices; ++i)
        {
            if (!visited[i])
            {
                stack<size_t> s;
                s.push(i);
                vector<size_t> parent(numVertices, size_t(-1)); // Vector to store parent of each vertex

                while (!s.empty())
                {
                    size_t v = s.top();
                    s.pop();

                    if (!visited[v])
                    {
                        visited[v] = true;

                        // Iterate over neighbors of vertex v
                        for (size_t neighbor = 0; neighbor != numVertices; ++neighbor)
                        {
                            if (g.getAdjacencyMatrix()[v][neighbor])
                            {
                                if (!visited[neighbor])
                                {
                                    s.push(neighbor);
                                    parent[neighbor] = v; // Set parent of neighbor
                                }
                                else if (parent[v] != neighbor)
                                {
                                    // Back edge detected, construct cycle
                                    string cycle;
                                    size_t u = v;
                                    while (u != neighbor)
                                    {
                                        cycle = to_string(u) + "->" + cycle; // Add parent vertex to cycle string
                                        u = parent[u];                       // Trace back to parent vertices
                                    }
                                    cycle = to_string(neighbor) + "->" + cycle + to_string(neighbor); // Add neighbor vertex again to close the cycle
                                    cout << "The cycle is: " << cycle << endl;
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
        }
        // No back edges detected, so there's no cycles
        return false;
    }

    string Algorithms::isBipartite(Graph g)
    {
        size_t numVertices = g.getNumVertices(); // Change type to size_t

        const vector<vector<int>> &adjMatrix = g.getAdjacencyMatrix();
        vector<size_t> colors(numVertices, static_cast<size_t>(-1)); // Vector to store vertex colors (-1 for uncolored, 0 and 1 for two colors)
        queue<size_t> q;
        vector<size_t> partA, partB;
        bool isBipartite = true;

        // Perform BFS traversal from each vertex
        for (size_t i = 0; i < numVertices; ++i) // Change loop variable to size_t
        {
            if (colors[i] == static_cast<size_t>(-1))
            {
                q.push(i);
                colors[i] = 0;      // Color vertex i with color 0
                partA.push_back(i); // Add vertex i to partition A

                while (!q.empty())
                {
                    size_t curr = q.front();
                    q.pop();

                    // Assign opposite color to neighbors
                    size_t nextColor = 1 - colors[curr];                          // Toggle the color (0 to 1 or 1 to 0)
                    for (size_t neighbor = 0; neighbor < numVertices; ++neighbor) // Change loop variable to size_t
                    {
                        if (adjMatrix[curr][neighbor])
                        {
                            // If neighbor is uncolored, assign the opposite color and enqueue it
                            if (colors[neighbor] == static_cast<size_t>(-1))
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
            for (size_t vertex : partA)
            {
                partition += to_string(vertex) + ", ";
            }
            partition.pop_back(); // Remove trailing comma and space
            partition.pop_back();
            partition += "}, B={";
            for (size_t vertex : partB)
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
        size_t numVertices = g.getNumVertices(); // Change type to size_t
        const vector<vector<int>> &adjMatrix = g.getAdjacencyMatrix();

        // Initialize distance array
        vector<int> dist(numVertices, INT_MAX);
        dist[0] = 0; // Set distance from source vertex to itself as 0

        // Relax edges repeatedly to find shortest paths
        for (size_t i = 0; i < numVertices - 1; ++i) // Change loop variable to size_t
        {
            for (size_t u = 0; u < numVertices; ++u) // Change loop variable to size_t
            {
                for (size_t v = 0; v < numVertices; ++v) // Change loop variable to size_t
                {
                    if (adjMatrix[u][v] && dist[u] != INT_MAX && dist[u] + adjMatrix[u][v] < dist[v])
                    {
                        dist[v] = dist[u] + adjMatrix[u][v];
                    }
                }
            }
        }

        // Check for negative cycles
        for (size_t u = 0; u < numVertices; ++u) // Change loop variable to size_t
        {
            for (size_t v = 0; v < numVertices; ++v) // Change loop variable to size_t
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
