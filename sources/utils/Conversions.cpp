#include "../../headers/utils/Conversions.h"

namespace Conversions {

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
