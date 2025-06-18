#include "ConstructPlane.h"
    
Mesh ConstructPlane::constructMesh(const GeomData& geomData) {
	if (!geomData.plane){
		std::cout << "[ConstructPlane] Input geometryData doesn't have any Plane" << std::endl;
		return Mesh();
	}
	else{
		return createRectangleMeshFromPlane(*geomData.plane);
	}
}

using namespace geom;

    
Mesh ConstructPlane::createRectangleMeshFromPlane(const PlaneData& planeData, float width, float height) {
    Mesh mesh;
    Vector normalCGAL = planeData.getNormal();
    Eigen::Vector3f normal(normalCGAL.x(), normalCGAL.y(), normalCGAL.z());
    normal.normalize();

    // Calcul du centre
    PointT originT = planeData.m_important_points[0];
    Eigen::Vector3f origin(originT.x, originT.y, originT.z);
    
    // Vecteurs dans le plan
    Eigen::Vector3f axis1;
    if (std::abs(normal.z()) < 0.9f) {
        axis1 = normal.cross(Eigen::Vector3f::UnitZ());
    } else {
        axis1 = normal.cross(Eigen::Vector3f::UnitX());
    }
    axis1.normalize();
    Eigen::Vector3f axis2 = normal.cross(axis1).normalized();

    // Coins du rectangle
    Eigen::Vector3f halfW = 0.5f * width * axis1;
    Eigen::Vector3f halfH = 0.5f * height * axis2;

    Eigen::Vector3f p1 = origin - halfW - halfH;
    Eigen::Vector3f p2 = origin + halfW - halfH;
    Eigen::Vector3f p3 = origin + halfW + halfH;
    Eigen::Vector3f p4 = origin - halfW + halfH;

    // Conversion Eigen -> CGAL::Point_3
    Point pt1(p1.x(), p1.y(), p1.z());
    Point pt2(p2.x(), p2.y(), p2.z());
    Point pt3(p3.x(), p3.y(), p3.z());
    Point pt4(p4.x(), p4.y(), p4.z());

    // Ajout des sommets
    auto v0 = mesh.add_vertex(pt1);
    auto v1 = mesh.add_vertex(pt2);
    auto v2 = mesh.add_vertex(pt3);
    auto v3 = mesh.add_vertex(pt4);

    // Ajout des faces
    mesh.add_face(v0, v1, v2);
    mesh.add_face(v0, v2, v3);

    return mesh;
}





Mesh ConstructPlane::createRectangleMeshFromPlaneFrontier(const PlaneData& planeData) {
    Mesh mesh;
    std::vector<PointT> frontier = planeData.m_important_points;
    // On supppose que les point sont bien orienté pour l'instant
    Point p0(frontier[0].x, frontier[0].y, frontier[0].z);
    Point p1(frontier[1].x, frontier[1].y, frontier[1].z);
    Point p2(frontier[2].x, frontier[2].y, frontier[2].z);
    Point p3(frontier[3].x, frontier[3].y, frontier[3].z);

    auto v0 = mesh.add_vertex(p0);
    auto v1 = mesh.add_vertex(p1);
    auto v2 = mesh.add_vertex(p2);
    auto v3 = mesh.add_vertex(p3);

    // Ajout des faces
    mesh.add_face(v0, v1, v2);
    mesh.add_face(v0, v2, v3);

    return mesh;
}
