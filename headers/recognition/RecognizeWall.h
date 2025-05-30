#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <iostream>

namespace PlaneRecognition {

class RecognizeWall {
public:
    using PointT = pcl::PointNormal;
    using PointCloudT = pcl::PointCloud<PointT>;

    // Constructeur
    RecognizeWall();

    // Charge un nuage de points
    void setInputCloud(const PointCloudT::Ptr& cloud);

    // Lance la reconnaissance de murs (plans dominants)
    void detectWalls();

    // Retourne les murs détectés comme un vecteur de nuages (un par mur)
    std::vector<PointCloudT::Ptr> getDetectedWalls() const;
    pcl::PointCloud<PointT>::Ptr getRemainingCloud() const;

private:
    PointCloudT::Ptr m_inputCloud;
    pcl::PointCloud<PointT>::Ptr m_remainingCloud;
    std::vector<PointCloudT::Ptr> m_detectedWalls;
};

}

