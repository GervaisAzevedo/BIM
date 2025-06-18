#include <CGAL/boost/graph/copy_face_graph.h>
#include "Conversions.h"

namespace Conversions {

SurfaceMesh polyhedronToSurfaceMesh(const Polyhedron& poly) {
    SurfaceMesh sm;
/*
    // Map from Polyhedron vertex handle to Surface_mesh vertex descriptor
    std::map<Polyhedron::Vertex_const_handle, SurfaceMesh::Vertex_index> vmap;

    // 1) Copier les sommets
    for (auto vit = poly.vertices_begin(); vit != poly.vertices_end(); ++vit) {
        SurfaceMesh::Vertex_index vi = sm.add_vertex(vit->point());
        vmap[vit] = vi;
    }

    // 2) Copier les faces
    for (auto fit = poly.facets_begin(); fit != poly.facets_end(); ++fit) {
        std::vector<SurfaceMesh::Vertex_index> face_vertices;

        // Parcours circulaire des sommets de la face
        auto h = fit->facet_begin();
        do {
            face_vertices.push_back(vmap[h->vertex()]);
            h = h->next();
        } while (h != fit->facet_begin());

        sm.add_face(face_vertices);
    }
*/
    return sm;
}

// Surface_mesh → Polyhedron
Polyhedron surfaceMeshToPolyhedron(const SurfaceMesh& mesh) {
    Polyhedron poly;
    
    if (!CGAL::is_triangle_mesh(mesh)) {
        std::cerr << "Erreur : le maillage n'est pas triangulaire, conversion impossible." << std::endl;
        return poly;
    }

    // Utilisation de copy_face_graph pour convertir entre structures différentes
    CGAL::copy_face_graph(mesh, poly);

    std::cout << "[DEBUG] Polyhedron a " << poly.size_of_facets() << " faces après conversion." << std::endl;
    return poly;
}


// Conversion PCL → CGAL (avec normales)
PS3 pclToCgal(const pcl::PointCloud<pcl::PointNormal>& pcl_cloud) {
    PS3 cgal_cloud;

    for (const auto& pt : pcl_cloud.points) {
        const Point cgal_point(pt.x, pt.y, pt.z);
        const Vector cgal_normal(pt.normal_x, pt.normal_y, pt.normal_z);

        auto idx = cgal_cloud.insert(cgal_point);
        cgal_cloud.normal(*idx) = cgal_normal;
    }

    return cgal_cloud;
}

// Conversion CGAL → PCL (avec normales)
pcl::PointCloud<pcl::PointNormal> cgalToPcl(const PS3& cgal_cloud) {
    pcl::PointCloud<pcl::PointNormal> pcl_cloud;

    for (auto it = cgal_cloud.begin(); it != cgal_cloud.end(); ++it) {
        const Point& pt = cgal_cloud.point(*it);
        const Vector& normal = cgal_cloud.normal(*it);

        pcl::PointNormal pcl_pt;
        pcl_pt.x = pt.x();
        pcl_pt.y = pt.y();
        pcl_pt.z = pt.z();
        pcl_pt.normal_x = normal.x();
        pcl_pt.normal_y = normal.y();
        pcl_pt.normal_z = normal.z();

        pcl_cloud.push_back(pcl_pt);
    }

    return pcl_cloud;
}
}
