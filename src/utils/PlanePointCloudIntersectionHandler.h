#pragma once

#include "GeometryData.h"
#include <limits>
#include <cmath>
#include <stdexcept>
#include <iostream>

using GeometryData = geom::GeometryData;
using Line = geom::Line;
using Vector = geom::Vector;
using Point = geom::Point;
using PointT = geom::PointT;
using PointCloudT = geom::PointCloudT;

struct PlanePointCloudIntersectionHandler {
    const GeometryData* g1 = nullptr;
    const GeometryData* g2 = nullptr;
    Line sharedEdge;
    Point minPt;
    Point maxPt;

    PlanePointCloudIntersectionHandler() = default;

    PlanePointCloudIntersectionHandler(const GeometryData* _g1,const GeometryData* _g2, const Line& edge)
        : g1(_g1), g2(_g2), sharedEdge(edge) {
        std::tie(minPt, maxPt) = computeExtremePointsOnSharedEdge();
    }


float getLength() const {
    return std::sqrt(CGAL::squared_distance(minPt, maxPt));
}

void print() const {
    std::cout << "Shared edge: "
              << "Direction(" << sharedEdge.direction().vector().x() << ", "
                              << sharedEdge.direction().vector().y() << ", "
                              << sharedEdge.direction().vector().z() << ")\n";

    std::cout << "Min point: (" << minPt.x() << ", " << minPt.y() << ", " << minPt.z() << ")\n";
    std::cout << "Max point: (" << maxPt.x() << ", " << maxPt.y() << ", " << maxPt.z() << ")\n";
}


std::pair<Point, Point> computeExtremePointsOnSharedEdge() {
    const auto& cloud1 = g1->getPointCloud();
    const auto& cloud2 = g2->getPointCloud();
    
    if ((!cloud1 || cloud1->empty()) && (!cloud2  || cloud2->empty())) {
        throw std::runtime_error("[PlanePointCloudIntersectionHandler] Empty point cloud in both of the studied planes for computeExtremePointsOnSharedEdge");
    }
    
    if (sharedEdge.direction().dz() < 0)
        sharedEdge = Line(sharedEdge.point(0), -sharedEdge.direction());
    
    const Vector direction = sharedEdge.direction().vector();
    const Point origin = sharedEdge.point(0);

    float minProj = std::numeric_limits<float>::infinity();
    float maxProj = -std::numeric_limits<float>::infinity();
    Point minPoint, maxPoint;
    
    auto processCloud = [&](const PointCloudT::Ptr& cloud) {
        for (const auto& pt : cloud->points) {
            Point p(pt.x, pt.y, pt.z);
            Point proj = sharedEdge.projection(p);
            
            Vector vec = proj - origin;
            float t = static_cast<float>(vec * direction / std::sqrt(direction.squared_length()));

            if (t < minProj) {
                minProj = t;
                minPoint = proj;
            }
            if (t > maxProj) {
                maxProj = t;
                maxPoint = proj;
            }
        }
    };
    
    if (cloud1 && !cloud1->empty()) {
        processCloud(cloud1);
    }
    if (cloud2  && !cloud2->empty()) {
        processCloud(cloud2);
    }

    return {minPoint, maxPoint};
}

};

