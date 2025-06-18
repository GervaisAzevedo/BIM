#pragma once

#include "../../utils/GeometryData.h"

class AdvancingFrontSurfaceReconstruction {
public:
    static geom::Mesh run(const geom::PS3& pointSet);
};

