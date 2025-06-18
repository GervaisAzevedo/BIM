#pragma once

#include "BimElementType.h"
#include "../utils/GeometryData.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>

#include <iostream>
#include <string>
#include <vector>

class BimElement {
protected:
    BimElementType m_type;
    geom::GeometryData m_geomData;

public:
    explicit BimElement(BimElementType type);
    BimElement(geom::GeometryData geomData, BimElementType type = BimElementType::NOT_ASSIGNED);
    virtual ~BimElement() = default;

    // === Getters ===
    BimElementType getType() const { return m_type; }
    // === Setters ===
    void setType(BimElementType type) { m_type = type; }
    void setGeometryData(const geom::GeometryData& geomData) { m_geomData = geomData; }
    void setGeometryData(geom::GeometryData&& geomData) { m_geomData = std::move(geomData); }
    geom::GeometryData& getGeometryData() { return m_geomData; }
const geom::GeometryData& getGeometryData() const { return m_geomData; }

};

