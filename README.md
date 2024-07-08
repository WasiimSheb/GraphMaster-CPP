
---

# GraphMaster-CPP: Advanced Graph Algorithms in C++

This repository contains projects and exercises for the System Programming B course, focusing on advanced graph algorithms and operations, implemented in C++.

## Features

- **Graph Algorithms**: Implementation of various graph algorithms.
- **Graph Operations**: Functions for graph manipulation and data retrieval.

## Installation

1. Clone the repository:
    ```bash
    git clone https://github.com/WasiimSheb/GraphMaster-CPP.git
    ```
2. Navigate to the project directory:
    ```bash
    cd GraphMaster-CPP
    ```

## Compilation

Compile the source code using the provided `Makefile`:
```bash
make
```

## Usage

Run the demo executable:
```bash
./Demo
```

## Files and Directories

- **Algorithms.cpp / Algorithms.hpp**: Implementation and header files for graph algorithms.
- **Graph.cpp / Graph.hpp**: Core implementation of the graph class.
- **Demo.cpp**: Demonstrates the usage of the graph library.
- **TestCounter.cpp / test.cpp**: Test files to validate functionality.
- **doctest.h**: Header file for the doctest framework used for testing.
- **Makefile**: Makefile for compiling the project.

### Graph Functions

- **PrintGraph**: Prints graph data, including vertices and edges.
- **LoadGraph**: Updates the adjacency matrix of the graph.
- **GetAdjacencyMatrix**: Returns the current adjacency matrix.
- **NumOfVertices**: Returns the number of vertices.

### Algorithms

- **isConnected**: Checks if the graph is connected.
- **shortestPath**: Finds the shortest path between two vertices using BFS.
- **isContainsCycle**: Detects cycles in the graph using DFS.
- **isBipartite**: Checks if the graph is bipartite using BFS.
- **negativeCycle**: Detects negative cycles using the Bellman-Ford algorithm.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request.


---
