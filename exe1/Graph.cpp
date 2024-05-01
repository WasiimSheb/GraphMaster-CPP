#include<iostream>

#include "Graph.hpp"

using namespace std;
namespace ariel{
    void Graph::printGraph(){
        cout << "{";
        for (size_t i = 0; i < this -> adjacencymatrix.size(); ++i){
            for (size_t j = 0; j < this -> adjacencymatrix[i].size(); ++j)
            {
                cout << this -> adjacencymatrix[i][j];
                if (i != this -> adjacencymatrix.size() - 1 || j != this -> adjacencymatrix.size() - 1 ){
                    cout << ", ";
                }
            }    
        }
        cout << "}" << endl;
    }

    void Graph::loadGraph(vector<vector<int>> adjmat){
        this -> adjacencymatrix.resize(adjmat.size());
        for (size_t i = 0; i < adjmat.size(); ++i) {
            this -> adjacencymatrix[i].resize(adjmat[i].size());
        }
        size_t count = this -> adjacencymatrix.size();
        for (size_t i = 0; i < count; ++i){
            for (size_t j = 0; j < count; ++j){
                this -> adjacencymatrix[i][j] = adjmat[i][j];
            }
        }
    }
    // because we're returning a reference to the user rather than a copy
    const vector<vector<int>>& Graph::getAdjacencyMatrix(){
    return adjacencymatrix;
}
    int Graph::getNumVertices(){
        return this -> adjacencymatrix.size();
    }
};

