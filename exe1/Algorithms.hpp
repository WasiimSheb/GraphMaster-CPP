#include "Graph.hpp"
namespace ariel{
    class Algorithms{
        public:
            static bool isConnected(Graph g);

            static string shortestPath(Graph g, int start, int end);

            static string isContainsCycle(Graph g);

            static bool isBipartite(Graph g);

    };
}
