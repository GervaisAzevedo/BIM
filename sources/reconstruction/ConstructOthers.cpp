#include "../../headers/reconstruction/ConstructOthers.h"

/*

#include <CGAL/advancing_front_surface_reconstruction.h>
#include <CGAL/Scale_space_surface_reconstruction_3.h>

ConstructOthers::Mesh ConstructOthers::reconstructMesh(const PS3& pointSet, Method method) {
    std::vector<Point> points;
    for (auto p : pointSet.points()) {
        points.push_back(p);
    }

    Mesh mesh;

    if (method == Method::ADVANCING_FRONT) {
        std::vector<std::array<std::size_t, 3>> triangles;
        CGAL::advancing_front_surface_reconstruction(points.begin(), points.end(), std::back_inserter(triangles));

        std::vector<Mesh::Vertex_index> vtx;
        for (const auto& p : points)
            vtx.push_back(mesh.add_vertex(p));

        for (const auto& t : triangles)
            mesh.add_face(vtx[t[0]], vtx[t[1]], vtx[t[2]]);
    } else if (method == Method::SCALE_SPACE) {
        using Reconstruction = CGAL::Scale_space_surface_reconstruction_3<Kernel>;
        using Tree = Reconstruction::Tree;

        Tree tree(points.begin(), points.end());
        tree.increase_scale(4); // nombre d’itérations (à ajuster)

        std::vector<std::array<std::size_t, 3>> triangles;
        CGAL::advancing_front_surface_reconstruction(tree.points().begin(), tree.points().end(), std::back_inserter(triangles));

        std::vector<Mesh::Vertex_index> vtx;
        for (const auto& p : tree.points())
            vtx.push_back(mesh.add_vertex(p));

        for (const auto& t : triangles)
            mesh.add_face(vtx[t[0]], vtx[t[1]], vtx[t[2]]);
    }

    return mesh;
}

*/
