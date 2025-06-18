#include "Bim.h"

Bim::Bim(std::vector<std::unique_ptr<BimElement>>&& elements) {
    classifyElements(std::move(elements));
}

void Bim::classifyElements(std::vector<std::unique_ptr<BimElement>>&& elements) {
    for (auto& e : elements) {
        switch (e->getType()) {
            case BimElementType::GROUND:
                m_ground_element = std::unique_ptr<GroundElement>(static_cast<GroundElement*>(e.release()));
                break;
            case BimElementType::WALL:
                m_wall_elements.push_back(std::unique_ptr<WallElement>(static_cast<WallElement*>(e.release())));
                break;
            case BimElementType::ROOF:
                m_roof_elements.push_back(std::unique_ptr<RoofElement>(static_cast<RoofElement*>(e.release())));
                break;
            case BimElementType::OTHER:
            default:
                m_other_elements.push_back(std::unique_ptr<OtherElement>(static_cast<OtherElement*>(e.release())));
                break;
        }
    }
}

void Bim::estimateGround() {
    if (m_wall_elements.size() < 2) {
        std::cerr << "[Bim] Not enough walls to estimate ground.\n";
        return;
    }
    Line normalToGround = m_wall_shared_edge_elements[0]->sharedEdge;
    Point point = m_wall_shared_edge_elements[0]->minPt;
    
    Plane groundPlane = normalToGround.perpendicular_plane(point);

    // Création du GeometryData
    GeometryData groundGeom;
    groundGeom.plane = PlaneData(groundPlane);  // If planeData is value
    groundGeom.plane->addImportantPoint(PointT(
        static_cast<float>(point.x()), 
        static_cast<float>(point.y()), 
        static_cast<float>(point.z())
    ));
    // Création de l'élément sol
    setGround(std::make_unique<GroundElement>(groundGeom));
    m_ground_element->printInfo();
    
}
    



bool Bim::areClose(const BimElement& e1, const BimElement& e2, float eps) {
    const auto& cloud1 = e1.getGeometryData().getPointCloud();
    const auto& cloud2 = e2.getGeometryData().getPointCloud();

    if (!cloud1 || !cloud2 || cloud1->empty() || cloud2->empty()) {
        return false;
    }

    // KD-tree pour recherche rapide dans cloud2
    pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
    kdtree.setInputCloud(cloud2);

    std::vector<int> indices(1);
    std::vector<float> distances(1);

    // Cherche un point de cloud1 proche d’un point de cloud2
    for (const auto& pt : *cloud1) {
        if (kdtree.nearestKSearch(pt, 1, indices, distances) > 0 && distances[0] <= eps * eps) {
            return true; // dès qu’un point proche est trouvé
        }
    }

    return false;
}



void Bim::constructAllMeshes() {
    WallMeshConstructor wallMeshConstructor;
    GenericMeshConstructor genMeshConstructor;
    GroundMeshConstructor groundMeshConstructor;

    if (m_ground_element)
        groundMeshConstructor.constructMesh(*m_ground_element);

    for (auto& e : m_wall_elements)
        wallMeshConstructor.createWallShape(*e, *m_ground_element); 
    for (auto& e : m_roof_elements)
        genMeshConstructor.constructMesh(*e);

    for (auto& e : m_other_elements)
        genMeshConstructor.constructMesh(*e);
}

void Bim::printBimInfo() const {
    std::cout << "BIM Information:\n";
    std::cout << "- Has Ground: " << (m_ground_element ? "Yes" : "No") << "\n";
    std::cout << "- Number of Wall Elements: " << m_wall_elements.size() << "\n";
    std::cout << "- Number of Shared Wall Elements: " << m_wall_shared_edge_elements.size() << "\n";
    std::cout << "- Number of Roof Elements: " << m_roof_elements.size() << "\n";
    std::cout << "- Number of Other Elements: " << m_other_elements.size() << "\n";
}

// Fonctions d'ajout

void Bim::setGround(std::unique_ptr<GroundElement> ground) {
    m_ground_element = std::move(ground);
}

void Bim::addWall(std::unique_ptr<WallElement> wall) {
    m_wall_elements.push_back(std::move(wall));
}

void Bim::addWallSharedEdge(std::unique_ptr<WallSharedEdgeElement> wall_shared_edge) {
    wall_shared_edge->w1->setHeight(wall_shared_edge->getLength());
    wall_shared_edge->w2->setHeight(wall_shared_edge->getLength());
    m_wall_shared_edge_elements.push_back(std::move(wall_shared_edge));
}

void Bim::addRoof(std::unique_ptr<RoofElement> roof) {
    m_roof_elements.push_back(std::move(roof));
}

void Bim::addOther(std::unique_ptr<OtherElement> other) {
    m_other_elements.push_back(std::move(other));
}

