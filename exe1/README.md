#Algorithms: 
--------------------------------------------------------------------------
  isConnected:
  
    Description: Determines whether the given graph is connected or not.
  
  Parameters:
  
    Graph g: The graph to be checked for connectivity.
    
  Returns:
  
    bool: Returns true if the graph is connected, false otherwise.
    
  /**************************************************************/
  
  shortestPath:
  
  Description: Finds the shortest path from a source vertex to a destination vertex using Breadth-First Search (BFS).
  
  Parameters:
  
  Graph g: The graph in which the shortest path is to be found.
  
  int start: The source vertex.
  
  int end: The destination vertex.
  
  Returns:
  
  string: Returns a string representing the shortest path from the source vertex to the destination vertex. If no path exists, it returns a message indicating that no path exists.
  
  /**************************************************************/
  
  isContainsCycle:
  
  Description: Determines whether the given graph contains a cycle or not using Depth-First Search (DFS).
  
  Parameters:
  
  Graph g: The graph to be checked for cycles.
  
  Returns:
  
  bool: Returns true if the graph contains a cycle, false otherwise.
  
  /**************************************************************/
  
  isBipartite:
  
  Description: Determines whether the given graph is bipartite or not using BFS.
  
  Parameters:
  
  Graph g: The graph to be checked for bipartiteness.
  
  Returns:
  
  string: Returns a string indicating whether the graph is bipartite and the two partitions if it is. If not bipartite, returns "0".
  
  /**************************************************************/
  
  negativeCycle:
  
    Description: Determines whether the given graph contains a negative cycle or not using Bellman-Ford algorithm.
    
    Parameters:
    
      Graph g: The graph to be checked for negative cycles.
    
    Returns:
    
      string: Returns a string indicating whether the graph contains a negative cycle or not.
