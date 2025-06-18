#include "../reconstruction/GroundMeshConstructor.h"
#include "../reconstruction/ConstructPlane.h"


void GroundMeshConstructor::constructMesh(BimElement& element) const {
        auto& geomData = element.getGeometryData();
       
       if (element.getGeometryData().plane) {
    geomData.setFineMesh(
        ConstructPlane::createRectangleMeshFromPlane(
            *element.getGeometryData().plane, 50.0f, 50.0f
        )
    );
}
}



