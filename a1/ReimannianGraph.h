#pragma once

class ReimannianGraph {

private:
	int ** g;
public:
	
	// We are representing an undirected wieghted graph here
	ReimannianGraph(int n_vertices) {
		
		// Allocate a adjacency matrix
		g = new int*[n_vertices];
		for (int i = 0; i < n_vertices; i++) {
			g[i] = new int[n_vertices];
			for (int j = 0; j < n_vertices; j++) {
				g[i][j] = 0;
			}
		}

	}

	
	void addEdge(int v1, int v2, int w) {
		g[v1][v2] = w;
		g[v2][v1] = w;
	}

	

};