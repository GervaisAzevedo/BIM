#pragma once

#include "BimElementType.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>

#include <iostream>





class BimElement {
protected:
    BimElementType m_type;
    geom::GeometryData geomData;


public:
    explicit BimElement(BimElementType type);
    virtual ~BimElement() = default;
    
    BimElementType getType() const { return m_type; }

};

