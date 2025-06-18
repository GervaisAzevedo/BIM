#pragma once

#include "../utils/GeometryData.h"
#include "../bim/BimElement.h"

class IElementMeshConstructor {
public:

    using Mesh = geom::Mesh;
    using GeomData = geom::GeometryData;
    
    virtual ~IElementMeshConstructor() = default;
    virtual void constructMesh(BimElement& element) const = 0;
};
