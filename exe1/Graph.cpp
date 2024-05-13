#include <iostream>
#include "Graph.hpp"

using namespace std;
using namespace ariel;

// Define the member functions of the Graph class

void Graph::printGraph(){
    int numofedges = 0;
        for (size_t i = 0; i < this -> adjacencymatrix.size(); ++i){
            for (size_t j = 0; j < this -> adjacencymatrix[i].size(); ++j)
            {
                if (adjacencymatrix[i][j] != 0){
                    numofedges++;
                }
            }  
        }
        cout << "Graph with "<< this -> adjacencymatrix.size() << " vertices and " << numofedges << " edges." << endl;
    }

// Load the graph from the adjacency matrix
void Graph::loadGraph(vector<vector<int>> adjmat)
{
    for (size_t i = 0; i < adjmat.size(); i++)
    {
        if (adjmat.size() != adjmat[i].size())
        {
            throw invalid_argument("Invalid graph: The graph is not a square matrix.");
        }
    }
    adjacencymatrix = adjmat;
}

// Get the adjacency matrix of the graph
const vector<vector<int>> &Graph::getAdjacencyMatrix()
{
    return adjacencymatrix;
}

// Get the number of vertices in the graph
size_t Graph::getNumVertices()
{
    return adjacencymatrix.size();
}
