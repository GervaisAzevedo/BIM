#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/IO/read_points.h>
#include <vector>
#include <array>
#include <iostream>

#include "AdvancingFrontSurfaceReconstruction.h"

// Types
using Kernel = geom::Kernel;
using Point = geom::Point;
using Vector = geom::Vector;
using Mesh = geom::Mesh;
using PS3 = geom::PS3;
using Facet = std::array<std::size_t, 3>;

struct Perimeter {
  double bound;

  Perimeter(double bound)
    : bound(bound)
  {}

  template <typename AdvancingFront, typename Cell_handle>
  double operator() (const AdvancingFront& adv, Cell_handle& c,
                     const int& index) const
  {
    if (bound == 0) {
      return adv.smallest_radius_delaunay_sphere(c, index);
    }

    double d = 0;
    d = sqrt(CGAL::squared_distance(c->vertex((index + 1) % 4)->point(),
                                    c->vertex((index + 2) % 4)->point()));
    if (d > bound) return adv.infinity();
    d += sqrt(CGAL::squared_distance(c->vertex((index + 2) % 4)->point(),
                                     c->vertex((index + 3) % 4)->point()));
    if (d > bound) return adv.infinity();
    d += sqrt(CGAL::squared_distance(c->vertex((index + 1) % 4)->point(),
                                     c->vertex((index + 3) % 4)->point()));
    if (d > bound) return adv.infinity();

    return adv.smallest_radius_delaunay_sphere(c, index);
  }
};

Mesh AdvancingFrontSurfaceReconstruction::run(const PS3& pointSet) {
  std::cout << "[AFSR] Début de la reconstruction..." << std::endl;

  const auto point_map = pointSet.point_map();
  const auto normal_map = pointSet.normal_map();

  std::vector<Facet> facets;

  double perimeter_bound = 0.0;
  double radius_ratio_bound = 5.0;

  Perimeter priority(perimeter_bound);

  std::cout << "[AFSR] Reconstruction avec CGAL::advancing_front_surface_reconstruction..." << std::endl;

  CGAL::advancing_front_surface_reconstruction(
    pointSet.points().begin(), pointSet.points().end(),
    std::back_inserter(facets),
    priority,
    radius_ratio_bound
  );

  std::cout << "[AFSR] Nombre de triangles générés : " << facets.size() << std::endl;

  // Création du mesh
  Mesh mesh;
  std::vector<Mesh::Vertex_index> vertex_indices;
  vertex_indices.reserve(pointSet.size());

  for (auto it = pointSet.begin(); it != pointSet.end(); ++it) {
    vertex_indices.push_back(mesh.add_vertex(get(point_map, *it)));
  }

  for (const auto& facet : facets) {
    mesh.add_face(vertex_indices[facet[0]],
                  vertex_indices[facet[1]],
                  vertex_indices[facet[2]]);
  }

  std::cout << "[AFSR] Reconstruction terminée." << std::endl;
  
  std::cout << "[AFSR] mesh has " << mesh.number_of_faces() << " faces and "
          << mesh.number_of_vertices() << " vertices." << std::endl;

  return mesh;
}

