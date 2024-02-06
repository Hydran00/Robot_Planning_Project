#include "planner/voronoi/utils/dijkstra.h"
// Prints shortest paths from src to all other vertices
void shortestPath(const std::vector<vector<pair<int, double>>> &adj, int V, int src) {

  std::cout << "Graph size: " << V << std::endl; 
  if(V == 0){
    return;
  }
  // Create a priority queue to store vertices that
  // are being preprocessed.
  priority_queue<Node, vector<Node>, greater<Node>> pq;

  // Create a vector for distances and initialize all
  vector<double> dist(V, numeric_limits<double>::max());

  // Insert source itself in priority queue and initialize
  // its distance as 0.
  pq.push(make_pair(0, src));
  dist[src] = 0.0;

  /* Looping till priority queue becomes empty (or all
  distances are not finalized) */
  while (!pq.empty()) {
    // The first vertex in pair is the minimum distance
    // vertex, extract it from priority queue.
    // vertex label is stored in second of pair (it
    // has to be done this way to keep the vertices
    // sorted distance (distance must be first item
    // in pair)
    int u = pq.top().second;
    pq.pop();

    // Get all adjacent of u.
    for (auto x : adj[u]) {
      // Get vertex label and weight of current
      // adjacent of u.
      int v = x.first;
      double weight = x.second;

      // If there is shorted path to v through u.
      if (dist[v] > dist[u] + weight) {
        // Updating distance of v
        dist[v] = dist[u] + weight;
        pq.push(make_pair(dist[v], v));
      }
    }
  }

  // Print shorteturn;est distances stored in dist[]
  cout << "Vertex Distance from Source\n";
  for (int i = 0; i < V; ++i) {
    cout<<i<<" \t\t "<< dist[i]<<endl;
  }
  return;
}

// }  // namespace dijkstra
