#pragma once

#include "../bim/BimElement.h"
#include "IElementMeshConstructor.h"


class WindowMeshConstructor : public IElementMeshConstructor {
public:
    void constructMesh(BimElement& element) const override;
};

