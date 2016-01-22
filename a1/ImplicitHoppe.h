#pragma once
#include <OpenGP/types.h>
#include <OpenGP/SurfaceMesh/SurfaceMesh.h>
#include "SurfaceMeshVerticesKDTree.h"

using namespace OpenGP;

class ImplicitHoppe{
    SurfaceMeshVerticesKDTree accelerator;
    SurfaceMesh::Vertex_property<Vec3> vpoints;
    SurfaceMesh::Vertex_property<Vec3> vnormals;
    
public:
    ImplicitHoppe(SurfaceMesh& cloud) : accelerator(cloud){
        vpoints = cloud.get_vertex_property<Vec3>("v:point");
        vnormals = cloud.get_vertex_property<Vec3>("v:normal");        
    }

    /// Evaluates implicit function at position p
    Scalar eval_implicit_at(const Vec3& p) const{
        // TASK: compute closest point
		SurfaceMesh::Vertex closest_idx = accelerator.closest_vertex(p);

        // TASK: compute point-to-plane distance
		Vec3 closest_point = vpoints[closest_idx];
		Vec3 closest_normal = vnormals[closest_idx];
		return p.squaredNorm() - std::pow(.99, 2);
		return (closest_point - p).dot(closest_normal);

    }
};
