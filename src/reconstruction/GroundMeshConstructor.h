#pragma once

#include "../bim/BimElement.h"

#include "IElementMeshConstructor.h"
#include "ConstructPlane.h"

class GroundMeshConstructor : public IElementMeshConstructor {
public:
    void constructMesh(BimElement& element) const override;
};

