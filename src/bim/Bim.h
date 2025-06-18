#pragma once

#include "../reconstruction/WallMeshConstructor.h"
#include "../reconstruction/GenericMeshConstructor.h"
#include "../reconstruction/GroundMeshConstructor.h"

#include "BimElement.h"
#include "WallElement.h"
#include "WallSharedEdgeElement.h"
#include "RoofElement.h"
#include "OtherElement.h"
#include "GroundElement.h"
#include "BimElementType.h"


#include <pcl/common/distances.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <vector>
#include <memory>

using GeometryData = geom::GeometryData;
using PointT = geom::PointT;
using Point = geom::Point;
using Vector = geom::Vector;
using PlaneData = geom::PlaneData;
using Plane = geom::Plane;

class Bim {
protected:

	// All elements :
	// Ground : m_ground_element represents the ground: the plane expression ax + by + cz + d gives us the normal (a,b,c) which is the walls upward directions 
    std::unique_ptr<GroundElement> m_ground_element;
    std::vector<std::unique_ptr<WallElement>> m_wall_elements;
    std::vector<std::unique_ptr<WallSharedEdgeElement>> m_wall_shared_edge_elements;
    std::vector<std::unique_ptr<RoofElement>> m_roof_elements;
    std::vector<std::unique_ptr<OtherElement>> m_other_elements;

public:
    Bim() = default;
    Bim(std::vector<std::unique_ptr<BimElement>>&& elements);

    // Interdiction de la copie
    Bim(const Bim&) = delete;
    Bim& operator=(const Bim&) = delete;

    // Déplacement autorisé
    Bim(Bim&&) = default;
    Bim& operator=(Bim&&) = default;

    void printBimInfo() const;
    void classifyElements(std::vector<std::unique_ptr<BimElement>>&& elements);
    Point estimateGroundPoint(Vector normal);
    void estimateGround();
    void constructAllMeshes();
    
    bool areClose(const BimElement& e1, const BimElement& e2, float eps = 0.2);

    // Ajout unitaire
    void setGround(std::unique_ptr<GroundElement> ground);
    void addWall(std::unique_ptr<WallElement> wall);
    void addWallSharedEdge(std::unique_ptr<WallSharedEdgeElement> wall_shared_edge);
    void addRoof(std::unique_ptr<RoofElement> roof);
    void addOther(std::unique_ptr<OtherElement> other);

    // Accès
    bool hasGround() const { return m_ground_element != nullptr; }
    const GroundElement* getGround() const { return m_ground_element.get(); }
    const std::vector<std::unique_ptr<WallElement>>& getWallElements() const { return m_wall_elements; }
    const std::vector<std::unique_ptr<RoofElement>>& getRoofElements() const { return m_roof_elements; }
    const std::vector<std::unique_ptr<OtherElement>>& getOtherElements() const { return m_other_elements; }
};

