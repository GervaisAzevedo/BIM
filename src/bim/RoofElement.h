
#pragma once

#include "BimElement.h"

class RoofElement : public BimElement {
public:
    RoofElement(const geom::GeometryData& dataWithPlane);
};

