#pragma once
#include <vector>
#include <memory>

#include "../utils/GeometryData.h"

#include "../bim/Bim.h"
#include "../bim/BimElement.h"
#include "../bim/WallElement.h"
#include "../bim/RoofElement.h"
#include "../bim/GroundElement.h"
#include "../bim/OtherElement.h"

#include "PlaneRecognizer.h"


using PointCloudT = geom::PointCloudT;
using Segment = geom::Segment;
    
class ElementRecognizer {
public:
    
    ElementRecognizer() = default;

    Bim createBimFromCloud(const PointCloudT& cloud);
    void recognizeFromCloud(const PointCloudT& cloud, Bim& bim);
    
    int getMaxNumberOfPlanesToDetect() const { return m_maxNumberOfPlanesToDetect; }
    void setMaxNumberOfPlanesToDetect(int maxNumber) { m_maxNumberOfPlanesToDetect = maxNumber; }
    
    double getClusterTolerance() const { return m_clusterTolerance; }
    void setClusterTolerance(double tolerance) { m_clusterTolerance = tolerance; }
    
    int getMinClusterSize() const { return m_minClusterSize; }
    void setMinClusterSize(int size) { m_minClusterSize = size; }
    
    void detectSharedWallEdges(Bim& bim);
    void detectWallBases(Bim& bim);
    
    void detectGround(Bim& bim);
    
    void detectOthers(const PointCloudT& cloud, Bim& bim);

private:
    int m_maxNumberOfPlanesToDetect = 5;
    double m_clusterTolerance = 0.2;
    int m_minClusterSize = 100;

    PointCloudT::Ptr detectPlanes(const PointCloudT& cloud, Bim& bim);    
};

