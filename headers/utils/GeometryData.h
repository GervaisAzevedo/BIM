#pragma once

#include <memory>
#include <optional>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Polyhedron_3.h>

#include "../utils/Conversions.h"

namespace geom {

    using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
    using Point = Kernel::Point_3;
    using Vector = Kernel::Vector_3;
    using PS3 = CGAL::Point_set_3<Point, Vector>;
    using Mesh = CGAL::Polyhedron_3<Kernel>;

    using PointT = pcl::PointNormal;
    using PointCloudT = pcl::PointCloud<PointT>;

    struct GeometryData {
        std::shared_ptr<PointCloudT> rawPoints;
        Mesh coarseMesh;
        Mesh fineMesh;

        GeometryData() = default;

        GeometryData(std::shared_ptr<PointCloudT> raw)
            : rawPoints(std::move(raw)) {}

        GeometryData(std::shared_ptr<PointCloudT> raw, const Mesh& coarse, const Mesh& fine)
            : rawPoints(std::move(raw)), coarseMesh(coarse), fineMesh(fine) {}

        bool hasPoints() const {
            return rawPoints && !rawPoints->empty();
        }

        void setFromPointSet(const PS3& pointSet) {
            rawPoints = std::make_shared<PointCloudT>(Conversions::cgalToPcl(pointSet));
        }

        void setCoarseMesh(const Mesh& mesh) {
            coarseMesh = mesh;
        }

        void setFineMesh(const Mesh& mesh) {
            fineMesh = mesh;
        }

        const Mesh& getCoarseMesh() const {
            return coarseMesh;
        }

        const Mesh& getFineMesh() const {
            return fineMesh;
        }
    };

    using GeometryPtr = std::shared_ptr<GeometryData>;

} // namespace geom

