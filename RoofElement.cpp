#include "RoofElement.h"
#include <stdexcept>

RoofElement::RoofElement(const geom::GeometryData& dataWithPlane)
    : BimElement(dataWithPlane, BimElementType::ROOF) {
    
    if (!m_geomData.hasPlane()) {
        throw std::invalid_argument("RoofElement requires a plane in its GeometryData");
    }
}

