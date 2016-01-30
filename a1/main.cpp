#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderCloud.h>
#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderVertexNormals.h>
#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderShaded.h>
#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderFlat.h>
#include "ArcballWindow.h"
#include "ReimannianGraph.h"
#include "internal/SurfaceMeshVerticesKDTree.h"
#include <stack>
#include <queue>

using namespace OpenGP;

extern void reconstruct(SurfaceMesh& cloud, SurfaceMesh& output, uint resolution);

// Configuration 
// -------------
#define K 8
// -------------



int main(int argc, char** argv){
	if (argc != 2) mFatal("application requires one parameter! e.g. sphere.obj");
	SurfaceMesh point_cloud;
	bool success = point_cloud.read(argv[1]);
	if (!success) mFatal() << "File not found: " << argv[1];

	ArcballWindow window("lab02_reconstruction", 640, 480);

	uint n_vertices = point_cloud.n_vertices();

	// HOMEWORK TASKS
	// 1) Compute normals 
	//    - fetch kNN (k~6-10) for each point
	//    - compute the covariance matrix
	//    - extract (unoriented) normal form the eigendecomposition
	//
	// 2) Re-orient normals coherently
	//    - build Reimannian graph and compute weights
	//    - construct minimal spanning tree
	//    - flip the normals by traversing the tree


	//Setup SurfaceMesh properties
	SurfaceMesh::Vertex_property<std::vector<SurfaceMesh::Vertex>> kNNs = point_cloud.add_vertex_property<std::vector<SurfaceMesh::Vertex>>("v:kNN");
	SurfaceMesh::Vertex_property<Vec3>vplane_c = point_cloud.add_vertex_property<Vec3>("v:plane_c");
	SurfaceMesh::Vertex_property<Vec3>vnormals = point_cloud.get_vertex_property<Vec3>("v:normal");

	// Create KD Tree
	SurfaceMeshVerticesKDTree accelerator = SurfaceMeshVerticesKDTree(point_cloud);

	// Loop over all point_cloud points
	for (const auto& vertex : point_cloud.vertices()){

		// Determine kNN
		Vec3 pos = point_cloud.position(vertex);
		std::vector<SurfaceMesh::Vertex> kNN = accelerator.kNN(pos, K);
		kNNs[(SurfaceMesh::Vertex)vertex] = kNN;

		Vec3 center(0, 0, 0);

		// Put kNN into a matrix
		Eigen::MatrixXf kNN_mat(K, 3);
		for (int i = 0; i < K; i++) {
			Vec3 pos = point_cloud.position(kNN[i]);
			center += pos; // keep track to compute center
			kNN_mat.row(i) = pos;
		}

		//compute the center of the plane
		center = center * (1.0 / (double)K);
		vplane_c[(SurfaceMesh::Vertex)vertex] = center;

		// compute covariance matrix of kNN_mat in eigen (inspiration from http://stackoverflow.com/questions/15138634/eigen-is-there-an-inbuilt-way-to-calculate-sample-covariance)
		Eigen::MatrixXf c = kNN_mat.rowwise() - kNN_mat.colwise().mean();
		Eigen::Matrix3f cov = (c.adjoint() * c) / (double)(K - 1);

		// Determine eigenvalues/vectors of covariance matrix
		Eigen::EigenSolver<Eigen::MatrixXf> es(cov);
		int idx;
		es.eigenvalues().real().minCoeff(&idx);
		Vec3 smallest_e = es.eigenvectors().real().col(idx);

		// This point's normal is the smallest eigenvector
		vnormals[(SurfaceMesh::Vertex)vertex] = smallest_e;
	}

	// create a graph containing all vertices
	Graph* g = new Graph(n_vertices);


	float max_z = FLT_MIN;
	int v = -1; // this will be the anchor for traversal of MST (highest z value)
	
	// loop over vertices
	for (const auto& vertex : point_cloud.vertices()) {

		// determine the highest z value
		if (point_cloud.position(vertex)[2] > max_z) {
			max_z = point_cloud.position(vertex)[2];
			v = vertex.idx();
		}

		// add weighted edge for each NN
		for (const auto& kNN : kNNs[(SurfaceMesh::Vertex)vertex]) {
			Vec3 n_i = vnormals[vertex];
			Vec3 n_j = vnormals[kNN];
			double weight = 1 - abs(n_i.dot(n_j));
			if (vertex.idx() != kNN.idx()) g->addEdge(vertex.idx(), kNN.idx(), (float)weight);
		}

	}

	//force to -z, as thats what makes the surface shading work
	vnormals[SurfaceMesh::Vertex(v)] = Vec3(0, 0, -1);

	// compute the MST and traverse DFS order
	Graph* MST = g->MST();
	std::stack<int> S;
	S.push(v);
	Vec3 last_normal = vnormals[SurfaceMesh::Vertex(v)];
	bool* disc = new bool[n_vertices]; //keep track of discovered vertices
	int* parent_of = new int[n_vertices]; // keep track of parents

	// initialize discovery array
	for (int j = 0; j < n_vertices; j++) {
		disc[j] = 0;
		parent_of[j] = -1;
	}
	
	// do traversal
	while (S.size() > 0) {

		v = S.top(); S.pop();
		if (!disc[v]) {
			disc[v] = true;

			// set previous normal to parent
			if (parent_of[v] >= 0) {
				last_normal = (Vec3)vnormals[SurfaceMesh::Vertex(parent_of[v])];
			}

			// flip normal if necessary
			Vec3 this_normal = (Vec3)vnormals[SurfaceMesh::Vertex(v)];
			if (last_normal.dot(this_normal) < 0.0) {
				vnormals[SurfaceMesh::Vertex(v)] *= -1;
			}

			// stack up more vertices to visit in MST
			for (int i = 0; i < n_vertices; i++) {
				if (MST->at(v, i) > -1) {
					parent_of[i] = v;
					S.push(i);
				}
			}
		}
	}

    ///--- Display the input
    SurfaceMeshRenderCloud points_gl(point_cloud);
    SurfaceMeshRenderVertexNormals normals_gl(point_cloud);
    window.scene.add(points_gl);
    window.scene.add(normals_gl);
    
    ///--- Compute reconstruction
    SurfaceMesh output;
    reconstruct(point_cloud, output, 30 /*grid resolution*/);
    output.update_face_normals();
    output.update_vertex_normals();

    ///--- Renders reconstruction 
    SurfaceMeshRenderFlat mesh_gl(output);
    // SurfaceMeshRenderShaded mesh_gl(output);
    window.scene.add(mesh_gl); 
    
    return window.run();
}
