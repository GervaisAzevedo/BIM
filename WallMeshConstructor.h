#pragma once

#include "../bim/BimElement.h"
#include "../bim/WallElement.h"
#include "../bim/GroundElement.h"

#include "IElementMeshConstructor.h"
#include "ConstructPlane.h"

using Vector = geom::Vector;
using Point = geom::Point;
using Mesh = geom::Mesh;

class WallMeshConstructor : public IElementMeshConstructor {
public:
    void constructMesh(BimElement& element) const override;
    void createWallShape(WallElement& wall, GroundElement& ground);
};

