#include "PlaneRecognizer.h"

using PointT = pcl::PointNormal;
using Vector = geom::Vector;
using PointCloudT = geom::PointCloudT;

PlaneRecognizer::PlaneRecognizer()
    : m_inputCloud(new PointCloudT) {}


void PlaneRecognizer::setInputCloud(const PointCloudT::Ptr& cloud) {
    std::cout << "[PlaneRecognizer] Input cloud set with " << cloud->size() << " points.\n";
    m_inputCloud = cloud;
}

void PlaneRecognizer::setupSegmentation() {
    m_seg.setOptimizeCoefficients(true);
    m_seg.setModelType(pcl::SACMODEL_PLANE);
    m_seg.setMethodType(pcl::SAC_RANSAC);
    m_seg.setDistanceThreshold(0.1);
    m_seg.setMaxIterations(1000);
}

geom::PlaneData PlaneRecognizer::PlaneDataFromCoefficients(const pcl::ModelCoefficients& coeffs) {
    return geom::PlaneData(coeffs.values[0], coeffs.values[1], coeffs.values[2], coeffs.values[3]);
}

void PlaneRecognizer::logPlaneDetection(int index, const pcl::ModelCoefficients::Ptr& coeffs, const pcl::PointIndices::Ptr& inliers) {
    std::cout << "[PlaneRecognizer] Plane " << index + 1 << " detected with " << inliers->indices.size() << " inliers.\n";
    std::cout << "    Coefficients: ";
    for (const auto& c : coeffs->values) std::cout << c << " ";
    std::cout << "\n";
}

PointCloudT::Ptr PlaneRecognizer::extractPlane(const pcl::PointIndices::Ptr& inliers) {
    PointCloudT::Ptr planeCloud(new PointCloudT);
    m_extract.setInputCloud(m_remainingCloud);
    m_extract.setIndices(inliers);
    m_extract.setNegative(false);
    m_extract.filter(*planeCloud);

    std::cout << "[PlaneRecognizer] Plane extracted with " << planeCloud->size() << " points.\n";
    return planeCloud;
}

PointCloudT::Ptr PlaneRecognizer::clusterLargestRegion(const PointCloudT::Ptr& cloud) {
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.2);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(cloud->size());
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    if (clusterIndices.empty()) return nullptr;

    auto& largest = *std::max_element(
        clusterIndices.begin(), clusterIndices.end(),
        [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
            return a.indices.size() < b.indices.size();
        });

    PointCloudT::Ptr result(new PointCloudT);
    for (int idx : largest.indices)
        result->push_back((*cloud)[idx]);

    std::cout << "[PlaneRecognizer] Largest cluster retained with " << result->size() << " points.\n";
    return result;
}

void PlaneRecognizer::updateRemainingCloud(const pcl::PointIndices::Ptr& inliers) {
    m_extract.setInputCloud(m_remainingCloud);
    m_extract.setIndices(inliers);
    m_extract.setNegative(true);

    pcl::PointCloud<PointT>::Ptr rest(new PointCloudT);
    m_extract.filter(*rest);
    std::cout << "[PlaneRecognizer] Remaining cloud has " << rest->size() << " points.\n";

    m_remainingCloud = rest;
}


void PlaneRecognizer::retainExteriorPlanePoints(PointCloudT::Ptr& cloud) {
    std::cout << "[PlaneRecognizer] Retain Concave Hull\n";

    std::vector<pcl::Vertices> hullPolygons;

    pcl::ConcaveHull<PointT> concaveHull;
    concaveHull.setInputCloud(cloud);
    concaveHull.setAlpha(0.8);

    PointCloudT::Ptr hullCloud(new PointCloudT);
    concaveHull.reconstruct(*hullCloud, hullPolygons);

    *cloud = *hullCloud; // on remplace le contenu de cloud

    std::cout << "[PlaneRecognizer] Concave Hull contains " << cloud->size() << " points.\n";
}

void PlaneRecognizer::detectPlanes(int maxNumberToDetect) {
    m_elements.clear();

    if (!m_inputCloud || m_inputCloud->empty()) {
        std::cout << "[PlaneRecognizer] Input cloud is empty or null. Aborting plane detection.\n";
        return;
    }

    std::cout << "[PlaneRecognizer] Starting plane detection...\n";
    m_remainingCloud = std::make_shared<PointCloudT>(*m_inputCloud);

    setupSegmentation(); // Configure `m_seg` et `m_extract`

    for (int i = 0; i < maxNumberToDetect && !m_remainingCloud->empty(); ++i) {
        std::cout << "[PlaneRecognizer] Detecting plane " << i + 1 << "...\n";

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        m_seg.setInputCloud(m_remainingCloud);
        m_seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            std::cout << "[PlaneRecognizer] No more planes detected. Stopping.\n";
            break;
        }

        logPlaneDetection(i, coefficients, inliers);

        PointCloudT::Ptr planeCloud = extractPlane(inliers);

        PointCloudT::Ptr filteredCloud = clusterLargestRegion(planeCloud);
        if (!filteredCloud) {
            std::cout << "[PlaneRecognizer] No valid cluster. Skipping.\n";
            continue;
        }

        retainExteriorPlanePoints(filteredCloud);
        GeomData planeGeom(filteredCloud, PlaneDataFromCoefficients(*coefficients));
        m_elements.push_back(std::make_unique<BimElement>(planeGeom));

        updateRemainingCloud(inliers);
    }

    std::cout << "[PlaneRecognizer] Plane detection finished. " << m_elements.size() << " plane(s) detected.\n";
}


PointCloudT PlaneRecognizer::detectImportantPointsOfPlane(int maxNumberToDetect, PointCloudT::Ptr& cloud, const geom::PlaneData& plane) {
    PointCloudT::Ptr currentCloud(new PointCloudT(*cloud));

    retainExteriorPlanePoints(currentCloud); // TODO : utiliser les points retournés ?
    
    pcl::SACSegmentation<PointT> seg;
    pcl::ExtractIndices<PointT> extract;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.2);
    seg.setMaxIterations(1000);

    for (int i = 0; i < maxNumberToDetect && !currentCloud->empty(); ++i) {
        std::cout << "\n=============================\n";
        std::cout << "[PlaneRecognizer] Detecting Line " << i + 1 << "...\n";

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        seg.setInputCloud(currentCloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            std::cout << "[PlaneRecognizer] No more lines detected. Stopping.\n";
            break;
        }

        std::cout << "[PlaneRecognizer] Line " << i + 1 << " detected with " 
                  << inliers->indices.size() << " inliers.\n";
        std::cout << "  Direction : (" << coefficients->values[0] << ", " 
                                        << coefficients->values[1] << ", " 
                                        << coefficients->values[2] << ")\n";
        std::cout << "  Point     : (" << coefficients->values[3] << ", " 
                                        << coefficients->values[4] << ", " 
                                        << coefficients->values[5] << ")\n";

        PointCloudT::Ptr lineCloud(new PointCloudT);
        extract.setInputCloud(currentCloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*lineCloud);

        std::cout << "[PlaneRecognizer] Extracted line has " << lineCloud->size() << " points.\n";

        extract.setNegative(true);
        PointCloudT::Ptr rest(new PointCloudT);
        extract.filter(*rest);

        std::cout << "[PlaneRecognizer] Remaining cloud has " << rest->size() << " points.\n";
        currentCloud = rest;
    }

    return PointCloudT(); // TODO : retourner un nuage utile ?
}

bool PlaneRecognizer::isWall(const BimElement& bimElement) {
    if (bimElement.getGeometryData().plane) {
        Vector normal = bimElement.getGeometryData().plane->getNormal();
        std::cout << "[PlaneRecognizer::isWall] normal = " << normal << " (test z = 0)\n";
        bool result = std::abs(normal.z()) < m_allowedError;
        std::cout << "[PlaneRecognizer] Is it a wall? " << (result ? "Yes" : "No") << "\n";
        return result;
    }
    std::cout << "[PlaneRecognizer] Is it a wall? No\n";
    return false;
}

bool PlaneRecognizer::isRoof(const BimElement& bimElement) {
    if (bimElement.getGeometryData().plane) {
        Vector normal = bimElement.getGeometryData().plane->getNormal();
        // Ex: on peut considérer que le toit est un plan presque horizontal (normale proche de (0,0,1))
        bool result = std::abs(normal.z() - 1.0f) < m_allowedError;
        std::cout << "[PlaneRecognizer] Is it a roof? " << (result ? "Yes" : "No") << "\n";
        return result;
    }
    return false;
}

/* ========== GETTERS ========== */

std::vector<std::unique_ptr<BimElement>> PlaneRecognizer::getDetectedPlanes() {
    return std::move(m_elements);
}

PointCloudT::Ptr PlaneRecognizer::getRemainingCloud() const {
    return m_remainingCloud;
}

