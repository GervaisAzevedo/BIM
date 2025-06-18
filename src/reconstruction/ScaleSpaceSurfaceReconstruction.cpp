
#include "ScaleSpaceSurfaceReconstruction.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
 
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/Timer.h>
 
#include <fstream>
#include <iostream>
 
using Mesh = geom::Mesh; 
using Kernel = geom::Kernel;
using Point = geom::Point;
using Reconstruction = CGAL::Scale_space_surface_reconstruction_3<Kernel>;

using Smoother = CGAL::Scale_space_reconstruction_3::Weighted_PCA_smoother< Kernel > ;
using Mesher = CGAL::Scale_space_reconstruction_3::Alpha_shape_mesher< Kernel >;
using Facet_iterator = Reconstruction::Facet_const_iterator;                    ;
 

Mesh ScaleSpaceSurfaceReconstruction::run(const geom::PS3& pointSet) {

  Reconstruction reconstruct;
  reconstruct.insert(pointSet.points().begin(), pointSet.points().end());
  reconstruct.increase_scale(4);
  reconstruct.reconstruct_surface();
  
 
  Reconstruction::Point_range smoothed(reconstruct.points());
  Reconstruction::Facet_range polygons(reconstruct.facets());
   
  CGAL::Polygon_mesh_processing::orient_polygon_soup(smoothed, polygons);
 
  Mesh mesh;
  CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(smoothed, polygons, mesh);
  
  return mesh;
}
