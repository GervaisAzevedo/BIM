#include <CGAL/poisson_surface_reconstruction.h>

#include <iostream>

#include "PoissonSurfaceReconstruction.h"

// Types
using Kernel = geom::Kernel;
using FT = Kernel::FT;
using Point = geom::Point;
using Vector = geom::Vector;
using Polyhedron = geom::Polyhedron;
using PS3 = geom::PS3;


using Point_map  = typename PS3::Point_map;
using Normal_map = typename PS3::Vector_map;


geom::Mesh PoissonSurfaceReconstruction::run(const geom::PS3& pointSet) {
    std::cout << "[Poisson] Début de la reconstruction." << std::endl;

    Polyhedron output_mesh;

    // Calcul de l'espacement moyen entre points, paramètre utile pour la reconstruction
    FT average_spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(
        pointSet.points(),  
        6
    );

    std::cout << "[Poisson] average_spacing : " << average_spacing << std::endl;

    Point_map point_map = pointSet.point_map();
    Normal_map normal_map = pointSet.normal_map();

    std::cout << "[Poisson] Debut de la vrai reconstruction : " << std::endl;

bool success = CGAL::poisson_surface_reconstruction_delaunay(
    pointSet.begin(),           // <-- utiliser begin()
    pointSet.end(),             // <-- utiliser end()
    point_map,
    normal_map,
    output_mesh,
    average_spacing*10
);

    if (success) {
        std::cout << "[Poisson] Reconstruction réussie." << std::endl;
        return Conversions::polyhedronToSurfaceMesh(output_mesh);
    } else {
        std::cerr << "[Poisson] Échec de la reconstruction." << std::endl;
        return geom::Mesh(); // Retourne un mesh vide
    }
}

