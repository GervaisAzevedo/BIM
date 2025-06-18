
#pragma once

#include "WallElement.h"
#include "../utils/PlanePointCloudIntersectionHandler.h"

struct WallSharedEdgeElement {
    WallElement* w1 = nullptr;
    WallElement* w2 = nullptr;
    Line sharedEdge;

    Point minPt;
    Point maxPt;

    WallSharedEdgeElement() = default;

    WallSharedEdgeElement(WallElement* _w1, WallElement* _w2, const Line& edge)
        : w1(_w1), w2(_w2), sharedEdge(edge)
    {
        if (!w1 || !w2) {
            throw std::invalid_argument("WallElement pointers must not be null.");
        }

        PlanePointCloudIntersectionHandler handler(
            &w1->getGeometryData(),
            &w2->getGeometryData(),
            sharedEdge
        );

        minPt = handler.minPt;
        maxPt = handler.maxPt;
    }

    float getLength() const {
        return std::sqrt(CGAL::squared_distance(minPt, maxPt));
    }

    void print() const {
        std::cout << "WallSharedEdgeElement between walls:\n";
        std::cout << "Min point: (" << minPt.x() << ", " << minPt.y() << ", " << minPt.z() << ")\n";
        std::cout << "Max point: (" << maxPt.x() << ", " << maxPt.y() << ", " << maxPt.z() << ")\n";
        std::cout << "Length: " << getLength() << std::endl;
    }
};

