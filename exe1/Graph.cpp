#include <iostream>
#include "Graph.hpp"

using namespace std;
using namespace ariel;

// Define the member functions of the Graph class

Graph::Graph(){} // An empty constructor
Graph::~Graph(){} // A destructor 

/**
 * @brief this function prints how many edges and vertices do exist in the graph
*/
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

/** @brief Load the graph from the adjacency matrix
 *  @param adjmat an adjacency matrix representing the graph we would like to load into
 *  @details the function also checks whther the graph is legal or not and throws an exception if it's not
*/
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
