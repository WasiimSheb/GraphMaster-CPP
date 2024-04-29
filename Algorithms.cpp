#include <queue>
#include "Algorithms.hpp"
#include "Graph.hpp"

namespace ariel {
    bool Algorithms::isConnected(Graph g) {
        const vector<vector<int>>& adjMatrix = g.getAdjacencyMatrix();
        int numVertices = adjMatrix.size();

        // Initialize visited array to track visited vertices
        vector<bool> visited(numVertices, false);

        // Perform BFS traversal from the first vertex
        queue<int> q;
        q.push(0); // Start from vertex 0
        visited[0] = true;

        // BFS traversal
        while (!q.empty()) {
            int curr = q.front();
            q.pop();
            for (int neighbor = 0; neighbor < numVertices; ++neighbor) {
                if (adjMatrix[curr][neighbor] && !visited[neighbor]) {
                    q.push(neighbor);
                    visited[neighbor] = true;
                }
            }
        }

        // Check if all vertices are visited
        for (bool v : visited) {
            if (!v) {
                return false; // Graph is disconnected
            }
        }
        return true; // Graph is connected
    }
}
