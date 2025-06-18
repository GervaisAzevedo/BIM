#include "BimElement.h"

BimElement::BimElement(BimElementType type)
    : m_type(type) {}

BimElement::BimElement( geom::GeometryData geomData, BimElementType type)
    : m_type(type), m_geomData(std::move(geomData)) {}

