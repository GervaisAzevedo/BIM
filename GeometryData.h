#pragma once


#include <memory>
#include <optional>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/squared_distance_3.h>
#include <cmath>

#include "Conversions.h"
#include "PlaneData.h" 

#include <memory>
#include <optional>

namespace geom {


    inline Vector normalize(const geom::Vector& v) {
        double len = std::sqrt(v.squared_length());
        if (len < 1e-8) {
            throw std::runtime_error("Cannot normalize a vector with near-zero length.");
        }
        return v / len;
    }

    using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
    using Point = Kernel::Point_3;
    using Line = Kernel::Line_3;
    using Segment = Kernel::Segment_3;
    using Direction = Kernel::Direction_3;
    using Vector = Kernel::Vector_3;
    using Plane = Kernel::Plane_3;
    using PS3 = CGAL::Point_set_3<Point, Vector>;
    using Mesh = CGAL::Surface_mesh<Point>;
    using Polyhedron = CGAL::Polyhedron_3<Kernel>;

    using PointT = pcl::PointNormal;
    using PointCloudT = pcl::PointCloud<PointT>;
    

    struct GeometryData {
        std::shared_ptr<PointCloudT> pointCloud;
        Mesh coarseMesh;
        Mesh fineMesh;
        std::optional<PlaneData> plane;

        // === Constructeurs ===

        GeometryData() = default;

        explicit GeometryData(std::shared_ptr<PointCloudT> cloud)
            : pointCloud(std::move(cloud)) {}

        GeometryData(std::shared_ptr<PointCloudT> cloud, const Mesh& coarse, const Mesh& fine)
            : pointCloud(std::move(cloud)), coarseMesh(coarse), fineMesh(fine) {}

        GeometryData(std::shared_ptr<PointCloudT> cloud, const PlaneData& p)
            : pointCloud(std::move(cloud)), plane(p) {}

        GeometryData(std::shared_ptr<PointCloudT> cloud, const Mesh& coarse, const Mesh& fine, const PlaneData& p)
            : pointCloud(std::move(cloud)), coarseMesh(coarse), fineMesh(fine), plane(p) {}

        explicit GeometryData(const PlaneData& p)
            : plane(p) {}

        // === Méthodes d’état ===

        bool hasPoints() const {
            return pointCloud && !pointCloud->empty();
        }

        bool hasFineMesh() const {
            return !fineMesh.is_empty();
        }

        bool hasCoarseMesh() const {
            return !coarseMesh.is_empty();
        }

        bool hasPlane() const {
            return plane.has_value();
        }

        // === Accès / mutation au nuage de points ===

        void setPointCloud(std::shared_ptr<PointCloudT> cloud) {
            pointCloud = std::move(cloud);
        }

        const std::shared_ptr<PointCloudT>& getPointCloud() const {
            return pointCloud;
        }

        void setFromPointSet(const PS3& pointSet) {
            pointCloud = std::make_shared<PointCloudT>(Conversions::cgalToPcl(pointSet));
        }

        // === Accès / mutation aux maillages ===

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



        // === AConversion dans GeometryData pour les stratégies de CGAL ===
        
        Polyhedron getFinePolyhedron() const {
            return Conversions::surfaceMeshToPolyhedron(fineMesh);
        }

        Polyhedron getCoarsePolyhedron() const {
            return Conversions::surfaceMeshToPolyhedron(coarseMesh);
        }

        void setFineMeshFromPolyhedron(const Polyhedron& poly) {
            fineMesh = Conversions::polyhedronToSurfaceMesh(poly);
        }

        void setCoarseMeshFromPolyhedron(const Polyhedron& poly) {
            coarseMesh = Conversions::polyhedronToSurfaceMesh(poly);
        }




    };

    using GeometryPtr = std::shared_ptr<GeometryData>;

} // namespace geom

