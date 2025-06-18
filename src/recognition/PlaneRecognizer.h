#pragma once

#include <CGAL/Plane_3.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <iostream>

#include <pcl/common/projection_matrix.h>
#include <pcl/common/common.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#include <iostream>

#include "../utils/GeometryData.h"
#include "../utils/PlaneData.h"

#include "../bim/BimElement.h"
#include "../bim/WallElement.h"

using GeomData = geom::GeometryData;
using PlaneData = geom::PlaneData;
    
using PointT = geom::PointT;
using PointCloudT = geom::PointCloudT;

using Vector = geom::Vector;
using Plane = geom::Plane;

class PlaneRecognizer {
public:

    // Constructeur
    PlaneRecognizer();

    void setupSegmentation();
    void setInputCloud(const PointCloudT::Ptr& cloud);
    geom::PlaneData PlaneDataFromCoefficients(const pcl::ModelCoefficients& coeffs);
    
    void retainExteriorPlanePoints(PointCloudT::Ptr& cloud);

    void logPlaneDetection(int index, const pcl::ModelCoefficients::Ptr& coeffs, const pcl::PointIndices::Ptr& inliers);
    
    PointCloudT::Ptr extractPlane(const pcl::PointIndices::Ptr& inliers);
    PointCloudT::Ptr clusterLargestRegion(const pcl::PointCloud<PointT>::Ptr& cloud); 
    
    void updateRemainingCloud(const pcl::PointIndices::Ptr& inliers);
    
    void detectPlanes(int maxNumberToDetect);
    
    PointCloudT detectImportantPointsOfPlane(int maxNumberToDetect, PointCloudT::Ptr& cloud, const geom::PlaneData& plane);

    bool isWall(const BimElement& bimElement);
    bool isRoof(const BimElement& bimElement);
	
    std::vector<std::unique_ptr<BimElement>> getDetectedPlanes();
    PointCloudT::Ptr getRemainingCloud() const;
    
private:

    float m_allowedError = 0.2; // Epsilon
    PointCloudT::Ptr m_inputCloud;
    std::vector<std::unique_ptr<BimElement>> m_elements;
    PointCloudT::Ptr m_remainingCloud;
    
    pcl::SACSegmentation<PointT> m_seg;
    pcl::ExtractIndices<PointT> m_extract;
};



