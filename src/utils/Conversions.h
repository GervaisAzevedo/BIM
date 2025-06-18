#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <CGAL/Surface_mesh.h>
#include <CGAL/Polyhedron_3.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_set_3.h>

namespace Conversions {

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point = Kernel::Point_3;
using Vector = Kernel::Vector_3;
using PS3 = CGAL::Point_set_3<Point, Vector>;

using SurfaceMesh = CGAL::Surface_mesh<Point>;
using Polyhedron = CGAL::Polyhedron_3<Kernel>;


SurfaceMesh polyhedronToSurfaceMesh(const Polyhedron& poly);

Polyhedron surfaceMeshToPolyhedron(const SurfaceMesh& mesh);

PS3 pclToCgal(const pcl::PointCloud<pcl::PointNormal>& pcl_cloud);

pcl::PointCloud<pcl::PointNormal> cgalToPcl(const PS3& cgal_cloud);

} // namespace Conversions

