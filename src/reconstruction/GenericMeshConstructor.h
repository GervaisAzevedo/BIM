#pragma once

#include "IElementMeshConstructor.h"
#include "../bim/BimElement.h"
#include "../utils/GeometryData.h"
#include <memory>

class GenericMeshConstructor : public IElementMeshConstructor {
public:
    enum class Method {
        POISSON,
        ADVANCING_FRONT,
        SCALE_SPACE
    };

    void constructMesh(BimElement& element) const override;

private:
    static geom::Mesh reconstructMesh(const geom::PS3& pointSet, Method method);
};

