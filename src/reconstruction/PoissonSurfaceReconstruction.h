#pragma once

#include "../../utils/GeometryData.h"
#include "../../utils/Conversions.h"

class PoissonSurfaceReconstruction {
public:
    static geom::Mesh run(const geom::PS3& pointSet);
};

