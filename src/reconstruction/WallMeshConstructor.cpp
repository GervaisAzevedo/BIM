#include "../reconstruction/WallMeshConstructor.h"


void WallMeshConstructor::constructMesh(BimElement& element) const {
}



void WallMeshConstructor::createWallShape(WallElement& wall, GroundElement& ground){
    auto& geomData = wall.getGeometryData();
    
    Mesh mesh;
    Vector wallUpwardDirection = ground.getUpwardVector();
    
    Vector vertical = wallUpwardDirection / std::sqrt(wallUpwardDirection.squared_length()) * wall.getHeight();
    
    Point p0 = wall.getBase().source();
    Point p1 = wall.getBase().target();
    Point p2 = p1 + vertical;
    Point p3 = p0 + vertical;
    

    auto v0 = mesh.add_vertex(p0);
    auto v1 = mesh.add_vertex(p1);
    auto v2 = mesh.add_vertex(p2);
    auto v3 = mesh.add_vertex(p3);

    // Ajout des faces
    mesh.add_face(v0, v1, v2);
    mesh.add_face(v0, v2, v3);
    
    geomData.setCoarseMesh(mesh);
}

