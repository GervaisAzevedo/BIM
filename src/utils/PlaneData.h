#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Plane_3.h>
#include <CGAL/intersections.h>
#include <pcl/point_types.h>
#include <cmath>
#include <optional>
#include <vector>

namespace geom {

    using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
    using PointT = pcl::PointNormal;
    using Point = Kernel::Point_3;
    using Line = Kernel::Line_3;
    using Vector = Kernel::Vector_3;
    using Plane = Kernel::Plane_3;
    
    struct PlaneData {
    public:
        Plane m_plane;
        std::vector<PointT> m_important_points;

        PlaneData() = default;

        PlaneData(float a, float b, float c, float d)
            : m_plane(a, b, c, d) {}

        PlaneData(const Plane& p)
            : m_plane(p) {}

        // === Getters ===
        Vector getNormal() const { return m_plane.orthogonal_vector(); }
        const Plane& getCGALPlane() const { return m_plane; }

        float evaluate(const PointT& p) const {
            return static_cast<float>(m_plane.a()) * p.x +
                   static_cast<float>(m_plane.b()) * p.y +
                   static_cast<float>(m_plane.c()) * p.z +
                   static_cast<float>(m_plane.d());
        }

        bool contains(const PointT& p, float epsilon = 1e-3f) const {
            return std::abs(evaluate(p)) < epsilon;
        }

        void addImportantPoint(PointT p) { m_important_points.push_back(p); }

        
        std::optional<Line> getIntersectionLine(const PlaneData& other) const {
        
            std::cout << "[PlaneData] getIntersectionLine called.\n";
            auto result = CGAL::intersection(m_plane, other.m_plane);
           
            if (const Line* line = std::get_if<Line>(&*result)) {
            
            	std::cout << "[PlaneData] devrait retourner *line.\n";
                return *line;
            }
            
            	std::cout << "[PlaneData] a retourné std::nullopt.\n";
            return std::nullopt;
        }
    };

} // namespace geom

