#pragma once

#include "BimElement.h"

#include "../utils/GeometryData.h"

using Point = geom::Point;
using Vector = geom::Vector;
using PlaneData = geom::PlaneData;
using Plane = geom::Plane;
using Line = geom::Line;
using Segment = geom::Segment;

class WallElement : public BimElement {
public:
    WallElement(const geom::GeometryData& dataWithPlane);
    std::optional<Line> getIntersectionLine(const BimElement& other) const;
    
    void setHeight(float height) { m_height = height; }
    void setBase(Segment wallBase) { m_wallBase = wallBase; }
    void setThickness(float thickness) { m_thickness = thickness; }
    
    Segment getBase() const { return m_wallBase; }
    float getHeight() const { return m_height; }
    float getThickness() const { return m_thickness; }
private: 
    Segment m_wallBase;
    float m_height;
    float m_thickness;
    
};

