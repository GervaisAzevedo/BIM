#pragma once

#include "BimElement.h"

using Vector = geom::Vector;

class GroundElement : public BimElement {
public:
    GroundElement(const geom::GeometryData& dataWithPlane);
    void printInfo();
    Vector getUpwardVector();

private:
    float m_size;
};

