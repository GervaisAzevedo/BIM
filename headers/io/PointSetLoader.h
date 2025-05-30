#pragma once

#include "../utils/GeometryData.h"
#include <string>
#include <fstream>
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/IO/read_xyz_points.h>


class PointSetLoader {
public:
    static bool loadPLY(const std::string& filename, geom::GeometryData& data);
    static bool loadXYZ(const std::string& filename, geom::GeometryData& data);
};

