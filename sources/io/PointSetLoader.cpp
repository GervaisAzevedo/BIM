#include "../../headers/io/PointSetLoader.h"
#include "../../headers/utils/Conversions.h"

bool PointSetLoader::loadPLY(const std::string& filename, geom::GeometryData& data) {
    std::ifstream stream(filename, std::ios::binary);
    geom::PS3 pointSet;

    if (stream && CGAL::IO::read_PLY(stream, pointSet)) {
        data.setFromPointSet(pointSet);
        return true;
    }
    return false;
}

bool PointSetLoader::loadXYZ(const std::string& filename, geom::GeometryData& data) {
    std::ifstream stream(filename);
    geom::PS3 pointSet;

    if (stream && CGAL::IO::read_XYZ(stream, pointSet)) {
        data.setFromPointSet(pointSet);
        return true;
    }
    return false;
}

