#pragma once

#include "BimElement.h"

class OtherElement : public BimElement {
public:
    OtherElement(const geom::GeometryData& dataWithoutPlane);
};

