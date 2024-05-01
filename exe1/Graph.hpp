#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <iostream>
#include <vector>
using namespace std;

namespace ariel{
    class Graph{
        private:
            vector<vector<int>> adjacencymatrix;

        public:
            void printGraph();
            void loadGraph(vector<vector<int>> adjmat);
            const vector<vector<int>> &getAdjacencyMatrix();
            int getNumVertices();
    };
}
#endif // GRAPH_HPP
