#pragma once

#include<float.h>

class Graph {

private:
	float ** g;
	int n;

	int min_idx(float* C, int n) {
		int toReturn = 0;
		float current = FLT_MAX;
		for (int i = 0; i < n; i++) {
			if (C[i] < FLT_MAX) toReturn = i;
		}
		return toReturn;
	}

	int idx_of(std::vector<int> Q, int k) {
		for (int i = 0; i < Q.size(); i++) {
			if (Q.at(i) == k) return i;
		}
		return -1;
	}

public:
	
	// We are representing an undirected wieghted graph here
	Graph(int n_vertices) {
		
		n = n_vertices;

		// Allocate a adjacency matrix
		g = new float*[n_vertices];
		for (int i = 0; i < n_vertices; i++) {
			g[i] = new float[n_vertices];
			for (int j = 0; j < n_vertices; j++) {
				g[i][j] = -1;
			}
		}

	}

	
	void addEdge(int v1, int v2, float w) {
		g[v1][v2] = w;
		g[v2][v1] = w;
	}

	float at(int i, int j) {
		return g[i][j];
	}

	Graph* MST() {

		// https://en.wikipedia.org/wiki/Prim%27s_algorithm

		Graph* F = new Graph(this->n);
		float* C = new float[this->n]; 
		int* E = new int[this->n];
		std::vector<int> Q;

		for (int i = 0; i < this->n; i++) {
			C[i] = FLT_MAX; //any number than max edge weight;
			E[i] = -1;
			Q.push_back(i);
		}

		// Repeast until Q is empty
		while (Q.size() > 0) {

			// Remove from Q the v with the smallest C[v]
			float current = FLT_MAX;
			int v = 0;
			int idx = 0;
			for (int i = 0; i < Q.size(); i++) {
				if (C[Q.at(i)] < current) {
					v = Q.at(i);
					idx = i;
				}
			}

			Q.erase(Q.begin() + idx);

			//Add v to F, 
			if (E[v] != -1) {
				// copy edge into output
				int i = E[v] / (this->n);
				int j = E[v] % (this->n);
				F->addEdge(i, j, this->g[i][j]);
			}

			
			for (int w = 0; w < this->n; w++)  {
				// for each edge vw
				if (this->g[v][w] != -1) {

					// if w in Q
					if (!(idx_of(Q, w) < 0)) {

						// if this edge is less cost than C[w]
						if (this->g[v][w] < C[w]) {
							C[w] = this->g[v][w];
							E[w] = v*(this->n) + w;
						}

					}

				}
			}

		}

		return F;
		

	}


};