#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_set_3.h>

namespace Conversions {

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point = Kernel::Point_3;
using Vector = Kernel::Vector_3;
using PS3 = CGAL::Point_set_3<Point, Vector>;

// Convertit un PCL PointCloud en CGAL Point_set_3
PS3 pclToCgal(const pcl::PointCloud<pcl::PointNormal>& pcl_cloud);

// (optionnel) Convertit un CGAL Point_set_3 en PCL PointCloud
pcl::PointCloud<pcl::PointNormal> cgalToPcl(const PS3& cgal_cloud);

}

