#include <queue>
#include "Algorithms.hpp"
#include "Graph.hpp"
#include <unordered_map>
#include <stack>

#define INT_MAX 99999

namespace ariel
{

    Algorithms::Algorithms() {} // Empty constructor

    Algorithms::~Algorithms() {} // Destructor

    /**
     * @brief Checks if the graph is connected.
     * @param g Graph object representing the graph.
     * @return A boolean indicating whether the graph is connected.
     * @details This function performs a breadth-first search (BFS) traversal starting from the first vertex to determine if all vertices in the graph are reachable.
     */
    bool Algorithms::isConnected(Graph g)
    {
        const vector<vector<int>> &adjMatrix = g.getAdjacencyMatrix();
        size_t numVertices = adjMatrix.size();

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

    /**
     * @brief Finds the shortest path between two vertices in the graph using Dijkstra's algorithm.
     * @param g Graph object representing the graph.
     * @param start The starting vertex of the path.
     * @param end The ending vertex of the path.
     * @return A string representing the shortest path from the start vertex to the end vertex and "-1" if there is no path.
     * @details This function implements Dijkstra's algorithm to find the shortest path between the given start and end vertices in a weighted graph.
     */
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
            if (curr == static_cast<size_t>(end))
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

    /**
     * @brief Determines if the graph contains a cycle.
     * @param Graph object representing the graph.
     * @return A boolean value indicating whether the graph contains a cycle and also a string indicating where the cycle is.
     * @details This function performs a depth-first search (DFS) traversal from each vertex of the graph. During the DFS traversal,
     * it tracks visited vertices and their parent vertices to detect back edges. If a back edge is encountered, indicating the presence
     * of a cycle, the function returns true. Otherwise, if no back edges are detected, it returns false, indicating the absence of a cycle.
     */
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

    /**
     * @brief Determines if the graph is bipartite and partitions its vertices into two sets.
     * @param g Graph object representing the graph.
     * @return A string indicating whether the graph is bipartite and the partitioning of vertices into two sets.
     * @details This function performs a breadth-first search (BFS) traversal from each vertex of the graph. During the BFS traversal,
     * it assigns colors (0 and 1) to the vertices such that adjacent vertices have different colors. If it's possible to assign colors
     * without any conflicts, the graph is bipartite.
     */
    string Algorithms::isBipartite(Graph g)
    {
        size_t numVertices = g.getNumVertices();

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

    /**
     * @brief Determines if the graph contains a negative cycle.
     * @param Graph object representing the graph.
     * @returns A string indicating whether the graph contains a negative cycle.
     * @details This function implements the Bellman-Ford algorithm to detect negative cycles in the graph.
     */
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
