#pragma once

#include "../utils/GeometryData.h"


#include <CGAL/Origin.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>


#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/Vertices.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <Eigen/Dense>


#include <iostream>
using PlaneData = geom::PlaneData;
using GeomData = geom::GeometryData;
using Mesh = geom::Mesh;
using Point = geom::Point;
using Vector = geom::Vector;

class ConstructPlane {   
public:
    static Mesh createRectangleMeshFromPlaneFrontier(const PlaneData& planeData);

    
    static Mesh createRectangleMeshFromPlane(const PlaneData& planeData, float width = 1.0f, float height = 1.0f);

    static Mesh constructMesh(const GeomData& geomData);
    
};

