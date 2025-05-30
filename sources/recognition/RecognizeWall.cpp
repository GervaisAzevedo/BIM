#include "../../headers/recognition/RecognizeWall.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <iostream>

namespace PlaneRecognition {

    using PointT = pcl::PointNormal;
RecognizeWall::RecognizeWall() : m_inputCloud(new PointCloudT) {}

void RecognizeWall::setInputCloud(const PointCloudT::Ptr& cloud) {
    std::cout << "[RecognizeWall] Input cloud set with " << cloud->size() << " points.\n";
    m_inputCloud = cloud;
}

void RecognizeWall::detectWalls() {
    m_detectedWalls.clear();
    if (!m_inputCloud || m_inputCloud->empty()) {
        std::cout << "[RecognizeWall] Input cloud is empty or null. Aborting wall detection.\n";
        return;
    }

    std::cout << "[RecognizeWall] Starting wall detection...\n";
    m_remainingCloud = pcl::PointCloud<PointT>::Ptr(new PointCloudT(*m_inputCloud));
    pcl::SACSegmentation<PointT> seg;
    pcl::ExtractIndices<PointT> extract;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.2); // à ajuster selon la précision
    seg.setMaxIterations(1000);

    int max_planes = 5; // nombre de murs à extraire max
    for (int i = 0; i < max_planes && !m_remainingCloud->empty(); ++i) {
        std::cout << "[RecognizeWall] Detecting wall " << i + 1 << "...\n";

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        seg.setInputCloud(m_remainingCloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            std::cout << "[RecognizeWall] No more planes detected. Stopping.\n";
            break;
        }

        std::cout << "[RecognizeWall] Plane " << i + 1 << " detected with " << inliers->indices.size() << " inliers.\n";
        std::cout << "    Coefficients: ";
        for (const auto& c : coefficients->values)
            std::cout << c << " ";
        std::cout << "\n";

        pcl::PointCloud<PointT>::Ptr wall(new PointCloudT);
        extract.setInputCloud(m_remainingCloud);
        extract.setIndices(inliers);
        extract.setNegative(false); // on garde les inliers = le mur
        extract.filter(*wall);

        m_detectedWalls.push_back(wall);
        std::cout << "[RecognizeWall] Wall " << i + 1 << " extracted with " << wall->size() << " points.\n";

        // On retire les inliers du nuage pour chercher les murs suivants
        extract.setNegative(true);
        pcl::PointCloud<PointT>::Ptr rest(new PointCloudT);
        extract.filter(*rest);
        std::cout << "[RecognizeWall] Remaining cloud has " << rest->size() << " points.\n";

        m_remainingCloud = rest;
    }

    std::cout << "[RecognizeWall] Wall detection finished. " << m_detectedWalls.size() << " walls detected.\n";
}

std::vector<RecognizeWall::PointCloudT::Ptr> RecognizeWall::getDetectedWalls() const {
    return m_detectedWalls;
}
 pcl::PointCloud<PointT>::Ptr RecognizeWall::getRemainingCloud() const {
    return m_remainingCloud;
}

}

