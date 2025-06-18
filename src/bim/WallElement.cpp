#include "WallElement.h"
#include <stdexcept>

WallElement::WallElement(const geom::GeometryData& dataWithPlane)
    : BimElement(dataWithPlane, BimElementType::WALL) {
    
    if (!m_geomData.hasPlane()) {
        throw std::invalid_argument("WallElement requires a plane in its GeometryData");
    }
}


std::optional<Line> WallElement::getIntersectionLine(const BimElement& other) const {
    std::cout << "[WallElement] getIntersectionLine called.\n";
    const auto& plane1_opt = m_geomData.plane;
    const auto& plane2_opt = other.getGeometryData().plane;

    if (!plane1_opt.has_value() || !plane2_opt.has_value()) {
        std::cout << "[WallElement] But one doesn't have a plane.\n";
        return std::nullopt;
    }

    return plane1_opt->getIntersectionLine(plane2_opt.value());
}

