#include "ElementRecognizer.h"

// Tu peux ajouter d'autres includes pour fenêtres/portes

Bim ElementRecognizer::createBimFromCloud(const PointCloudT& cloud) {
    std::cout << "[ElementRecognizer] Creating Bim... \n";
    Bim bim;
    recognizeFromCloud(cloud, bim);    
    std::cout << "[ElementRecognizer] Bim Created. \n";
    return bim;
}

void ElementRecognizer::recognizeFromCloud(const PointCloudT& cloud, Bim& bim) {
    std::cout << "[ElementRecognizer] Recognizing elements \n";
    auto remainingCloud = detectPlanes(cloud, bim);
    detectGround(bim);
    detectWallBases(bim);
    detectOthers(*remainingCloud, bim);
    
}


PointCloudT::Ptr ElementRecognizer::detectPlanes(const PointCloudT& cloud, Bim& bim) {
    PlaneRecognizer recognizer;
    recognizer.setInputCloud(cloud.makeShared());
    recognizer.detectPlanes(m_maxNumberOfPlanesToDetect);

    for (auto& plane : recognizer.getDetectedPlanes()) {
    	if (recognizer.isWall(*plane)){
    	    auto wall = std::make_unique<WallElement>(plane->getGeometryData());
    	    bim.addWall(std::move(wall));
        }
        else if (recognizer.isRoof(*plane)){
    	    auto roof = std::make_unique<RoofElement>(plane->getGeometryData());
    	    bim.addRoof(std::move(roof));
        }
        else{
    	    auto other = std::make_unique<OtherElement>(plane->getGeometryData());
    	    bim.addOther(std::move(other));
        }
    }
    
    detectSharedWallEdges(bim);
    
    return recognizer.getRemainingCloud();
}

void ElementRecognizer::detectSharedWallEdges(Bim& bim){

    const auto& walls = bim.getWallElements();
    for (size_t i = 0; i < walls.size(); ++i) {
        for (size_t j = i + 1; j < walls.size(); ++j) {
        
            WallElement* w1 = walls[i].get();
            WallElement* w2 = walls[j].get();
    	    
            std::optional<Line> sharedEdge = w1->getIntersectionLine(*w2);
            if(sharedEdge.has_value())
            
            if(sharedEdge.has_value() && bim.areClose(*w1,*w2)){
            
                auto edgeInfo = std::make_unique<WallSharedEdgeElement>(w1, w2, *sharedEdge);
                
                edgeInfo->print();
                bim.addWallSharedEdge(std::move(edgeInfo));
                
            }
        }
    }
}


void ElementRecognizer::detectWallBases(Bim& bim){
    const auto& walls = bim.getWallElements();
    const auto& ground = bim.getGround();
    for (size_t i = 0; i < walls.size(); ++i) {
        WallElement* wall = walls[i].get();
        
        std::optional<Line> sharedEdge = wall->getIntersectionLine(*ground);
        auto edgeInfo = std::make_unique<PlanePointCloudIntersectionHandler>(
    &wall->getGeometryData(), &ground->getGeometryData(), *sharedEdge);
        
        if(sharedEdge.has_value()){
            Segment wallBase = Segment(edgeInfo->minPt, edgeInfo->maxPt);
            wall->setBase(wallBase);
        }
    }          
}


void ElementRecognizer::detectOthers(const PointCloudT& cloud, Bim& bim) {
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud.makeShared());

    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(m_clusterTolerance);   
    ec.setMinClusterSize(m_minClusterSize);       
    ec.setMaxClusterSize(cloud.size());
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud.makeShared());
    ec.extract(clusterIndices);

    int clusterId = 0;
    for (const auto& indices : clusterIndices) {
        PointCloudT::Ptr cluster(new PointCloudT);
        for (int idx : indices.indices)
            cluster->push_back(cloud.at(idx));

        geom::GeometryData geom;
        geom.pointCloud = cluster;
        auto other = std::make_unique<OtherElement>(geom);
        bim.addOther(std::move(other));

        std::cout << "[ElementRecognizer] Other cluster #" << clusterId++
                  << " with " << cluster->size() << " points added.\n";
    }

    if (clusterIndices.empty()) {
        std::cout << "[ElementRecognizer] No Other clusters found.\n";
    }
}

void ElementRecognizer::detectGround(Bim& bim) {
    bim.estimateGround();
}

