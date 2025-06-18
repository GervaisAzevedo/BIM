#include "GroundElement.h"
#include <stdexcept>

GroundElement::GroundElement(const geom::GeometryData& dataWithPlane)
    : BimElement(dataWithPlane, BimElementType::GROUND) {
    
    if (!m_geomData.hasPlane()) {
        throw std::invalid_argument("GroundElement requires a plane in its GeometryData");
    }
}


void GroundElement::printInfo() {
    std::cout << "=== GroundElement Info ===" << std::endl;
    std::cout << "Size: " << m_size << std::endl;

    const auto& geomData = getGeometryData(); // hérité de BimElement ?
    if (geomData.plane.has_value()) {
        const geom::PlaneData& planeData = geomData.plane.value();
        const auto& plane = planeData.getCGALPlane();
        const auto& normal = planeData.getNormal();

        std::cout << "--- PlaneData ---" << std::endl;
        std::cout << "Plane equation: "
                  << plane.a() << "x + "
                  << plane.b() << "y + "
                  << plane.c() << "z + "
                  << plane.d() << " = 0" << std::endl;

        std::cout << "Normal vector: ("
                  << normal.x() << ", "
                  << normal.y() << ", "
                  << normal.z() << ")" << std::endl;

        std::cout << "Important points (" << planeData.m_important_points.size() << "):" << std::endl;
        for (const auto& pt : planeData.m_important_points) {
            std::cout << "- ("
                      << pt.x << ", "
                      << pt.y << ", "
                      << pt.z << ")" << std::endl;
        }

    } else {
        std::cout << "No plane data available." << std::endl;
    }
}


Vector GroundElement::getUpwardVector(){
    auto n = m_geomData.plane->getNormal();
    n = (n.z() < 0) ? -n : n;
    return n;
}
