#include "Graph.hpp"
namespace ariel{
    class Algorithms{

        // private:
        //     // Helper function declaration for DFS traversal
        //     static string DFS(Graph& g, int start);

        public:
            static bool isConnected(Graph g);

            static string shortestPath(Graph g, int start, int end);
            // this function checks whether there is a cycle in the graph or not. if there is it prints
            // the cycle is: for example 1 -> 2 -> 3. and if there is not it simply returns 0
            static  bool isContainsCycle(Graph g);

            // this function checks whether a graph is isBipartite or not, returning the partiotion of the graph to two parts if possible 
            static string isBipartite(Graph g);

            // this function returns whether there is a negative cycle in the graph or not
            static string negativeCycle(Graph g);

    };
}
