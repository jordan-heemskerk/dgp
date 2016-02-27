#include "Decimator.h"
#include <OpenGP/MLogger.h>

/// is the collapse of halfedge h allowed? (check for manifold, foldovers, etc...)
bool Decimator::is_collapse_legal(Halfedge h){
    // tests for collapse candidate: v0 ---> v1
    Vertex v0 = mesh.from_vertex(h);
    Vertex v1 = mesh.to_vertex(h);

    
    if (mesh.is_deleted(h)) return false;
    /// TASK: check if h would cause a non-manifold change (bad!!!)
    /// hint: search the SurfaceMesh documentation
    if (!mesh.is_collapse_ok(h)) return false;
    
    /// TASK: what do you do if you have boundaries?
    /// note: this part is optional (use watertight input!)
    if( false /*.....*/ )
        return false;

    /// TEST: check for face foldovers
    bool causes_foldover = false;
    Point old_p0 = vpoints[v0]; ///< used to undo collapse simulation
    vpoints[v0] = vpoints[v1]; ///< simulates the collapse
    {
        /*for (auto && face : mesh.faces(v0)) {
            auto simulated = mesh.compute_face_normal(face);
            auto original = fnormals[face];
            
            if (simulated == Eigen::Vector3f::Zero()) continue;
            
            auto cos_of_dihedral = -simulated.dot(original)/ (simulated.norm() * original.norm());
            
            if (cos_of_dihedral >= min_cos) causes_foldover = true;
        }*/

		for (auto && he : mesh.halfedges(v0)) {
			SurfaceMesh::Face current_face = mesh.face(he);
			SurfaceMesh::Face next_face = mesh.face(mesh.opposite_halfedge(mesh.next_halfedge(he)));

			auto current_normal = mesh.compute_face_normal(current_face);
			auto next_normal = mesh.compute_face_normal(next_face);

			auto cos_of_dihedral = - current_normal.dot(next_normal) / (current_normal.norm() * next_normal.norm() );

			if (cos_of_dihedral >= min_cos) causes_foldover = true;

		}
        /// TASK: Check that the (post-collapse) faces have cos(dihedral)<min_cos
        /// note: decimation would still run without this!
        /// hint: see SurfaceMesh::compute_face_normal()
    }
    vpoints[v0] = old_p0; ///< undo simulation
    if(causes_foldover) 
        return false;

    return true; ///< all tests passed!
}

/// what is the priority of collapsing the halfedge h?
Scalar Decimator::halfedge_collapse_cost(Halfedge h){
    /// TASK: Compute the priority (quadric error) for collapsing halfedge "h"  
    /// hint: See what the class Quadric provides
    /// hint: Get/set the quadric of a vertex v by calling quadrics[v]
    auto v_i = mesh.from_vertex(h);
    auto v_j = mesh.to_vertex(h);
    auto Q_i = vquadrics[v_i];
    auto Q_j = vquadrics[v_j];
    auto Q_new = Q_i + Q_j;
    auto cost = Q_new.evaluate(vpoints[v_j]);

    return cost;
}

/// Find smallest half-edge collapse for vertex and (potentially) add it to the queue
void Decimator::enqueue_vertex(Vertex v){
    Halfedge best_halfedge; ///< invalid!
    Scalar best_halfedge_cost = inf();
    
    /// TASK
    /// 1) find smallest error out-going halfedge collapse
    /// 2) add the best halfedge to the priority queue
    
    for (auto && h : mesh.halfedges(v)) {
        if (halfedge_collapse_cost(h) < best_halfedge_cost) {
            best_halfedge_cost = halfedge_collapse_cost(h);
            best_halfedge = h;
        }
    }
    
    if (is_collapse_legal(best_halfedge)) queue.insert_or_update(best_halfedge, best_halfedge_cost);
}

void Decimator::update_quadric(Vertex vertex) {
    
    vquadrics[vertex].clear();
    for (auto&& face : mesh.faces(vertex)) {
        vquadrics[vertex] += Quadric(fnormals[face], vpoints[vertex]);
    }
}

/// Initialization
void Decimator::init(){
    // why? because face normals are needed to compute the initial quadrics
    mesh.update_face_normals();

    /// TASK: initialize (per-vertex) quadrics
    /// 1) quadric at vertex is the sum of the quadrics of its incident triangles
    /// hint: see Quadric.h and store the quadric in the vertex property vquadrics[]
    
    /// TASK: traverse all vertices and initialize the priority queue
    /// hint: Decimator::enqueue_vertex is to be used here
    
    for (auto&& vertex : mesh.vertices()) {

        update_quadric(vertex);
        Decimator::enqueue_vertex(vertex);
    }

}

void Decimator::exec(unsigned int target_n_vertices){
    mLogger() << "Decimator::exec" << mesh.n_vertices() << target_n_vertices << "in progress...";

    while (mesh.n_vertices()>target_n_vertices && !queue.is_empty()){
        // performing collapse v0 ---> v1
        Halfedge h = queue.pop();        
        Vertex v0 = mesh.from_vertex(h);
        Vertex v1 = mesh.to_vertex(h);

        /// TASK: main execution logic
        /// 1) check if this collapse is legal
        if (!is_collapse_legal(h)) continue;
        /// 2) perform the halfedge collapse (see docs)
        mesh.collapse(h);
        /// 3) update the quadric of v1
        Decimator::update_quadric(v1);
        /// 4) re-compute the collapse costs in neighborhood of v1
        for (auto && out_edge : mesh.halfedges(v1)) {
            enqueue_vertex(mesh.to_vertex(out_edge));
        }
        enqueue_vertex(v1);
        
        
        // just some debug output (for long processes)
        if (mesh.n_vertices() % 1000 == 0)
            mLogger() << "#v:" << mesh.n_vertices();
    }
}
