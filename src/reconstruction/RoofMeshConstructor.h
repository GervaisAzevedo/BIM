#pragma once

#include "../bim/BimElement.h"
#include "IElementMeshConstructor.h"


class RoofMeshConstructor : public IElementMeshConstructor {
public:
    void constructMesh(BimElement& element) const override;
};

