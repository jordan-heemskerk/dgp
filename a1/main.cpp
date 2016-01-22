#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderCloud.h>
#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderVertexNormals.h>
#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderShaded.h>
#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderFlat.h>
#include "ArcballWindow.h"
#include "ReimannianGraph.h"
#include "internal/SurfaceMeshVerticesKDTree.h"

using namespace OpenGP;

extern void reconstruct(SurfaceMesh& cloud, SurfaceMesh& output, uint resolution);

// Configuration 
// -------------
#define K 6
// -------------



int main(int argc, char** argv){
    if(argc!=2) mFatal("application requires one parameter! e.g. sphere.obj");
    SurfaceMesh point_cloud;
    bool success = point_cloud.read(argv[1]);
    if(!success) mFatal() << "File not found: " << argv[1];
  
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
    

    SurfaceMesh::Vertex_property<std::vector<SurfaceMesh::Vertex>> kNNs = point_cloud.add_vertex_property<std::vector<SurfaceMesh::Vertex>>("v:kNN");
	SurfaceMesh::Vertex_property<Vec3>vnormals = point_cloud.get_vertex_property<Vec3>("v:normal");

	SurfaceMeshVerticesKDTree accelerator = SurfaceMeshVerticesKDTree(point_cloud);
    for (const auto& vertex : point_cloud.vertices()){
        Vec3 pos = point_cloud.position(vertex);
        std::vector<SurfaceMesh::Vertex> kNN = accelerator.kNN(pos, K);
        kNNs[(SurfaceMesh::Vertex)vertex] = kNN;

        Eigen::MatrixXf kNN_mat(K, 3);

        for (int i = 0; i < K; i++) {
			Vec3 pos = point_cloud.position(kNN[i]);
			kNN_mat.row(i) = pos;
        }
        //std::cout << kNN_mat << std::endl << std::endl;
		// compute covariance matrix of kNN_mat in eigen (inspiration from http://stackoverflow.com/questions/15138634/eigen-is-there-an-inbuilt-way-to-calculate-sample-covariance)
		Eigen::MatrixXf c = kNN_mat.rowwise() - kNN_mat.colwise().mean();
		Eigen::Matrix3f cov = (c.adjoint() * c) / (double)(K - 1);

		Eigen::EigenSolver<Eigen::MatrixXf> es(cov);
		int idx;
		es.eigenvalues().real().minCoeff(&idx);
		Vec3 n = es.eigenvectors().real().col(idx);
		vnormals[(SurfaceMesh::Vertex)vertex] = n;
    } 

 

	ReimannianGraph* g = new ReimannianGraph(n_vertices);

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
