/***************************************************************************
 *   Copyright (C) 2006 - 2016 by                                          *
 *      Tarek Taha, KURI  <tataha@tarektaha.com>                           *
 *      Randa Almadhoun   <randa.almadhoun@kustar.ac.ae>                   *
 *                                                                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/
#include "cscpp/coverage_path_planning_heuristic_gpu.h"

namespace SSPP
{

CoveragePathPlanningHeuristic::CoveragePathPlanningHeuristic(ros::NodeHandle & nh, std::string collisionCheckModelP, std::string occlusionCullingModelN, bool d, bool gradualV, int hType)
{

    loadOBJFile(collisionCheckModelP.c_str(), modelPoints, triangles);
    cgalTree             = new Tree1(triangles.begin(),triangles.end());
    occlussionCulling    = new OcclusionCullingGPU(nh, occlusionCullingModelN);
    debug                = d;
    gradualVisualization = gradualV;
    heuristicType        = hType;
    treePub              = nh.advertise<visualization_msgs::Marker>("search_tree", 10);
    coveredPointsPub     = nh.advertise<sensor_msgs::PointCloud2>("gradual_coverage", 100);;
    pathPointPub         = nh.advertise<visualization_msgs::Marker>("path_point" , 10);
    pathPub              = nh.advertise<visualization_msgs::Marker>("path_testing", 10);
    octomapPub           = nh.advertise<octomap_msgs::Octomap>("octomap", 1);
    hullPub              = nh.advertise<visualization_msgs::Marker>("hull", 10);
    selectedPosePub      = nh.advertise<geometry_msgs::PoseArray>("selected_pt_poses",1);
    sensorPosesPub       = nh.advertise<geometry_msgs::PoseArray>("selected_sensor_poses",1);

    accuracySum          = 0.0;
    extraCovSum          = 0.0;
    extraAreaSum         = 0.0;
    volumetricVoxelRes   = 0.5;
    accW                 = 0.8;
    distW                = 0.5;
    covW                 = 1;
    angleW               = 0.2;
    selectedPointsNum    = 0;
    voxelResForConn      = 0.5;
    maxConnRadius        = std::sqrt((4.5*4.5) + (4.5*4.5)) + 0.01;
    //area
//     Triangles aircraftCGALT ;
//     meshSurface->loadOBJFile(collisionCheckModelP.c_str(), modelPoints, aircraftCGALT);
//     aircraftArea = meshSurface->calcCGALMeshSurfaceArea(aircraftCGALT);

    //octomap
    octomap::OcTree fullModelTree(volumetricVoxelRes);
    octomap::Pointcloud octPointCloud;
    for(int i = 0;i<occlussionCulling->cloud->points.size();i++)
    {
        octomap::point3d endpoint((float) occlussionCulling->cloud->points[i].x,(float) occlussionCulling->cloud->points[i].y,(float) occlussionCulling->cloud->points[i].z);
        octPointCloud.push_back(endpoint);
    }
    octomap::KeySet freeKeys,occupiedKeys;
    fullModelTree.computeUpdate(octPointCloud,octomap::point3d(0,0,0),freeKeys,occupiedKeys,-1);
    occupiedVoxelsNum = occupiedKeys.size();
    modelVolume = occupiedKeys.size()*(fullModelTree.getResolution()*fullModelTree.getResolution()*fullModelTree.getResolution());
    modelTotalEntroby = occupiedKeys.size()*(-1*(0.5)*(log(0.5)/log(2)));//unknown is 0.5

    //voxelgrid
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setInputCloud (occlussionCulling->cloud);
    voxelgrid.setLeafSize (volumetricVoxelRes, volumetricVoxelRes, volumetricVoxelRes);
    voxelgrid.filter(modelVoxels);

    //voxelgrid occlusion estimation
    originalCloudFilteredVoxels.setInputCloud (occlussionCulling->cloud);
    originalCloudFilteredVoxels.setLeafSize (voxelResForConn, voxelResForConn, voxelResForConn);
    originalCloudFilteredVoxels.initializeVoxelGrid();
    originalCloudFilteredVoxels.filter(modelVoxelsForConn);
}
double CoveragePathPlanningHeuristic::pointCloudDiff(pcl::PointCloud<pcl::PointXYZ>::Ptr globalCloudPtr, pcl::PointCloud<pcl::PointXYZ>::Ptr& pointCloudDiffPtr )
{
    pcl::PointCloud<pcl::PointXYZ> coveredVoxels;

    pcl::VoxelGridOcclusionEstimationGPU coveredCloudFilteredVoxels;
    coveredCloudFilteredVoxels.setInputCloud (globalCloudPtr);
    coveredCloudFilteredVoxels.setLeafSize (voxelResForConn, voxelResForConn, voxelResForConn);
    coveredCloudFilteredVoxels.initializeVoxelGrid();
    coveredCloudFilteredVoxels.filter(coveredVoxels);
    std::cout<<" ////// Coverage % : "<<((double)coveredVoxels.size()/(double)modelVoxelsForConn.size())*100<<" //////" <<std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr diffPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr diffPtrNoColor(new pcl::PointCloud<pcl::PointXYZ>);
    //iterate through the entire coverage grid to check the number of matched voxel between the original and the covered ones
    Eigen::Vector3i  min_b = originalCloudFilteredVoxels.getMinBoxCoordinates ();
    Eigen::Vector3i  max_b = originalCloudFilteredVoxels.getMaxBoxCoordinates ();
    int extraNum=0;
    for (int kk = min_b.z (); kk <= max_b.z (); ++kk)
    {
        for (int jj = min_b.y (); jj <= max_b.y (); ++jj)
        {
            for (int ii = min_b.x (); ii <= max_b.x (); ++ii)
            {

                Eigen::Vector3i ijk (ii, jj, kk);
                int index1 = originalCloudFilteredVoxels.getCentroidIndexAt (ijk);
                if(index1!=-1)
                {
                    Eigen::Vector4f centroid = originalCloudFilteredVoxels.getCentroidCoordinate (ijk);
                    Eigen::Vector3i ijk_in_Original= coveredCloudFilteredVoxels.getGridCoordinates(centroid[0],centroid[1],centroid[2]) ;

                    int index = coveredCloudFilteredVoxels.getCentroidIndexAt (ijk_in_Original);

                    if(index==-1)
                    {
                        pcl::PointXYZRGB point = pcl::PointXYZRGB(0,244,0);
                        pcl::PointXYZ pt ;
                        point.x = centroid[0];
                        pt.x = centroid[0];
                        point.y = centroid[1];
                        pt.y = centroid[1];
                        point.z = centroid[2];
                        pt.z = centroid[2];
                        diffPtr->points.push_back(point);
                        diffPtrNoColor->points.push_back(pt);
                        extraNum++;
                    }
                }
            }
        }
    }
    //    std::cout<<"number of diff cloud: "<<diffPtrNoColor->points.size()<<std::endl;
    //    std::cout<<"number of diff cloud: "<<diffPtr->points.size()<<std::endl;
    pointCloudDiffPtr->points = diffPtrNoColor->points;
    ///////////check the difference percentage///////////////
    double uncoveredPercent = ((double) diffPtr->size()/ (double) modelVoxelsForConn.size()) *100;
    return uncoveredPercent;
}

void CoveragePathPlanningHeuristic::clusteringPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZ> >& clustersPointCloudVec, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudDiffPtr)
{
    //divide the un covered parts into subregions (using euclidean clustering)
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr Kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
    Kdtree->setInputCloud (pointCloudDiffPtr);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.8); // 30cm
    ec.setMinClusterSize (5);
    ec.setMaxClusterSize (50000);//10000
    ec.setSearchMethod (Kdtree);
    ec.setInputCloud (pointCloudDiffPtr);
    ec.extract (cluster_indices);
    std::cout<<"number of clusters: "<<cluster_indices.size()<<std::endl;

    //////////converting the clusters to pointcloud/////////////////
    int j = 0;
    int r =0,g=0,b=0;
//    std::vector<pcl::PointCloud<pcl::PointXYZ> > clusters_pointcloud; //vector to store clusters
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredClusters (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (pointCloudDiffPtr->points[*pit]); //*
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      clustersPointCloudVec.push_back(*cloud_cluster);
      r+=10; b+=30; g+=20; //coloring
      for(int i=0; i<cloud_cluster->points.size(); i++)
      {
          pcl::PointXYZRGB point = pcl::PointXYZRGB(r,g,b);
          point.x = cloud_cluster->points[i].x;
          point.y = cloud_cluster->points[i].y;
          point.z = cloud_cluster->points[i].z;
          coloredClusters->points.push_back(point);
      }
//      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
      j++;
    }
}
void CoveragePathPlanningHeuristic::findClusterOuterPoints(geometry_msgs::PoseArray waypoints, pcl::PointCloud<pcl::PointXYZ>& cloudHull)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
    for(int i =0; i<waypoints.poses.size(); i++)
    {
        pcl::PointXYZ pt;
        pt.x = waypoints.poses[i].position.x;
        pt.y = waypoints.poses[i].position.y;
        pt.z = waypoints.poses[i].position.z;
        inputCloudPtr->push_back(pt);
    }
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud (inputCloudPtr);
    chull.setAlpha (1.0);
    chull.reconstruct (cloudHull);
    std::vector<geometry_msgs::Point> hullPoints;
    for (int i=0; i<cloudHull.size(); i++)
    {
        geometry_msgs::Point pt;
        pt.x = cloudHull.points[i].x;
        pt.y = cloudHull.points[i].y;
        pt.z = cloudHull.points[i].z;
        hullPoints.push_back(pt);
    }
    visualization_msgs::Marker hullMarker = drawPoints(hullPoints, 1,1000000);
    hullPub.publish(hullMarker);
}
void CoveragePathPlanningHeuristic::findClusterBB(pcl::PointCloud<pcl::PointXYZ> clusterPoints, geometry_msgs::Vector3& gridSize, geometry_msgs::Pose& gridStart)
{
    //finding bounding box of cluster
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterPointsPtr (new pcl::PointCloud<pcl::PointXYZ>);
    clusterPointsPtr->points = clusterPoints.points;
    pcl::PointCloud<pcl::PointXYZ> tempClusterPoints;
    pcl::VoxelGridOcclusionEstimationGPU grid;
    grid.setInputCloud (clusterPointsPtr);
    grid.setLeafSize (voxelResForConn, voxelResForConn, voxelResForConn);
    grid.initializeVoxelGrid();
    grid.filter(tempClusterPoints);

    //getting grid size and start
    Eigen::Vector4f min_b = grid.getCentroidCoordinate (grid.getMinBoxCoordinates());
    Eigen::Vector4f max_b = grid.getCentroidCoordinate (grid.getMaxBoxCoordinates ());

    // 3 and 5 is used to making the BB bigger not exactly on the boundry of the cluster
    // (sometimes it is very small set of samples and the descritization sample will not fit)
    double maximizeSizeXY = 5;
    double maximizeSizeZ = 3;
    gridSize.x = std::abs(max_b[0]-min_b[0]) + maximizeSizeXY;//5
    gridSize.y = std::abs(max_b[1]-min_b[1]) + maximizeSizeXY;//5
    gridSize.z = std::abs(max_b[2]-min_b[2]) + maximizeSizeZ;//3

    gridStart.position.x = min_b[0] - double(maximizeSizeXY/2);//5
    gridStart.position.y = min_b[1] - double(maximizeSizeXY/2);//5
    gridStart.position.z = min_b[2];//to avoid going under 0, UAVs can't fly under 0

}

CoveragePathPlanningHeuristic::~CoveragePathPlanningHeuristic()
{
    delete occlussionCulling;
}

bool CoveragePathPlanningHeuristic::terminateConditionReached(Node *node)
{
    double deltaCoverage;
    deltaCoverage = coverageTarget - node->coverage;
    selectedPointsNum++;

    std::cout<<"\n\n ****************** Total Coverage %: "<<node->coverage<<"  f = "<<node->f_value <<" **************************"<<std::endl;
    std::cout<<"\n\nAverage Accuracy per viewpoint is "<<accuracySum/accuracyPerViewpointAvg.size()<<std::endl;
    std::cout<<"Average extra coverage per viewpoint is "<<extraCovSum/extraCovPerViewpointAvg.size()<<std::endl;
    std::cout<<"Average extra Area per viewpoint is "<<extraAreaSum/extraAreaperViewpointAvg.size()<<std::endl;
    std::cout<<"number of viewpoints is "<<selectedPointsNum<<std::endl;
    std::cout<<"total Entropy is "<<node->totalEntroby<<"\n\n"<<std::endl;

    if (debug)
        std::cout<<"Delta Coverage:"<<deltaCoverage<<"\n";

    if ( deltaCoverage <= coverageTolerance)
    {
        std::cout<<"\n\nAverage Accuracy per viewpoint is "<<accuracySum/accuracyPerViewpointAvg.size()<<std::endl;
        std::cout<<"Average extra coverage per viewpoint is "<<extraCovSum/extraCovPerViewpointAvg.size()<<std::endl;
        std::cout<<"Average extra Area per viewpoint is "<<extraAreaSum/extraAreaperViewpointAvg.size()<<"\n\n"<<std::endl;

        return true;
    }
    else
    {
        if(gradualVisualization)
            displayGradualProgress(node);
        return false;
    }
}

bool CoveragePathPlanningHeuristic::isConnectionConditionSatisfied(geometry_msgs::Pose temp, geometry_msgs::Pose S)
{
    //collision check
    int intersectionsCount=0;
    //parent
    Point a(temp.position.x , temp.position.y ,temp.position.z );
    //child
    Point b(S.position.x, S.position.y, S.position.z);
    Segment seg_query(a,b);
    intersectionsCount = cgalTree->number_of_intersected_primitives(seg_query);
    if(intersectionsCount==0)
        return true;
    else
        return false;
}

bool CoveragePathPlanningHeuristic::isConnectionConditionSatisfied(SearchSpaceNode *temp, SearchSpaceNode *S)
{
    //collision check
    int intersectionsCount=0;
    //parent
    Point a(temp->location.position.x , temp->location.position.y ,temp->location.position.z );
    //child
    Point b(S->location.position.x, S->location.position.y, S->location.position.z);
    Segment seg_query(a,b);
    intersectionsCount = cgalTree->number_of_intersected_primitives(seg_query);
    if(intersectionsCount==0)
        return true;
    else
        return false;
}
bool CoveragePathPlanningHeuristic::isFilteringConditionSatisfied(geometry_msgs::Pose pose, geometry_msgs::PoseArray& correspondingSensorPoses, double minDist, double maxDist, pcl::PointCloud<pcl::PointXYZ>& globalCloud, std::vector<pcl::PointCloud<pcl::PointXYZ> >& accuracyClusters, double accuracyThreshhold)
{
    //model-node collision based filtering
    int intersectionsCount=0;
    Point a(pose.position.x ,pose.position.y ,pose.position.z);
    // Some Random point in arbitrary orientation
    Point b(100.0, 10.0, 56.0);
    Ray ray_query(a,b);
    intersectionsCount = cgalTree->number_of_intersected_primitives(ray_query);
    //    std::cout << "intersections: "<<intersectionsCount<< " intersections(s) with ray query" << std::endl;

    // the node is considered inside the model if the number of intersections is odd
    if(intersectionsCount%2 != 1)
    {
        //distance based filtering
        FT sqd = cgalTree->squared_distance(a); //consumes time but it is needed
//        std::cout << "sqd: "<< sqd << std::endl;

        if (sqd >=(minDist*minDist) && sqd <= (maxDist*maxDist) )
        {
            //coverage based filtering
            pcl::PointCloud<pcl::PointXYZ> sensorsCloud;
            for(int i=0; i<correspondingSensorPoses.poses.size(); i++)
            {
                pcl::PointCloud<pcl::PointXYZ> pts;
                pts = occlussionCulling->extractVisibleSurface(correspondingSensorPoses.poses[i]);
                sensorsCloud += pts;
                if(pts.size()>5)
                {
                    globalCloud += pts;
                    double accuracy = occlussionCulling->calcAvgAccuracy(pts,correspondingSensorPoses.poses[i]);
                    //std::cout<<"\naccuracy : "<<accuracy<<std::endl;
                    if(accuracy >= accuracyThreshhold)
                    {
                        accuracyClusters.push_back(pts);
                    }
                }

            }
            //to check if at least one of the sensors is processed and add the point or not
            if(sensorsCloud.size()>5)
                return true;
            else return false;

        } else return false;

    } else return false;
}

void CoveragePathPlanningHeuristic::displayProgress(vector<Tree> tree)
{

        geometry_msgs::Pose child;
        std::vector<geometry_msgs::Point> lineSegments;
        geometry_msgs::Point linePoint;
        for(unsigned int k=0;k<tree.size();k++)
        {
            for(int j=0;j<tree[k].children.size();j++)
            {
                child = tree[k].children[j];

                linePoint.x = tree[k].location.position.x;
                linePoint.y = tree[k].location.position.y;
                linePoint.z = tree[k].location.position.z;
                lineSegments.push_back(linePoint);
                linePoint.x = child.position.x;
                linePoint.y = child.position.y;
                linePoint.z = child.position.z;
                lineSegments.push_back(linePoint);

            }
        }
        visualization_msgs::Marker linesList = drawLines(lineSegments,1,6,1000000,0.08);
        treePub.publish(linesList);
}

bool CoveragePathPlanningHeuristic::isCost()
{
    if(heuristicType==InfoGainVolumetricH)
        return true;
    else return false;
}
void CoveragePathPlanningHeuristic::setMaxMinSensorAccuracy(std::vector<geometry_msgs::PoseArray> sensorsViewsSS)
{
    this->occlussionCulling->SSMaxMinAccuracy(sensorsViewsSS);
    maxDepth = std::sqrt(occlussionCulling->maxAccuracyError/0.0000285);
}
void CoveragePathPlanningHeuristic::setCoverageTarget(double coverageTarget)
{
    this->coverageTarget = coverageTarget;
}

void CoveragePathPlanningHeuristic::setCoverageTolerance(double coverageTolerance)
{
    this->coverageTolerance = coverageTolerance;
}

void CoveragePathPlanningHeuristic::setDebug(bool debug)
{
    this->debug = debug;
}

int CoveragePathPlanningHeuristic::getHeuristicType()
{
   return heuristicType;
}

//%TODO: this function will be cleaned later, it includes a lot of repetitions and conditions
void CoveragePathPlanningHeuristic::calculateHeuristic(Node *node)
{
    //    std::cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>"<<std::endl;

    if(node==NULL)
        return;


        pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ> visibleCloud, collectiveCloud;
        occlussionCulling->cloud->points =occlussionCulling->cloudCopy->points;
        //    float localViewEntroby =0.0;
        if(heuristicType != InfoGainVolumetricH)
        {
            for(int i=0; i<node->senPoses.size(); i++)
            {
                pcl::PointCloud<pcl::PointXYZ> temp;
                temp = occlussionCulling->extractVisibleSurface(node->senPoses[i].p);
                visibleCloud += temp;
                //        localViewEntroby += occlussionCulling->viewEntropy; //old way using raytracing inside voxel occlusion estimation
            }
            //    visibleCloud = occlussionCulling->extractVisibleSurface(node->senPose.p);
        }
    if(node->parent)
    {

        node->h_value  = 0;
        node->g_value  = 0;

        double f=0,d,c,a,dPar;

        // distance
        d = Dist(node->pose.p,node->parent->pose.p);
        dPar = maxConnRadius - d;
        node->distance = node->parent->distance + d;

        if(debug)
        {
            std::cout<<"parent distance :"<<node->parent->distance<<" current node distance: "<<node->distance<<"\n";
            std::cout<<"Calculated local distance d:"<<d<<" comulative distance: "<<node->distance<<"\n";
        }


        if(d!=0.0){
            //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
            if(heuristicType==SurfaceCoverageH || heuristicType==SurfaceCoveragewithOrientationH)
            {
                //coverage
                collectiveCloud.points = node->parent->cloud_filtered->points;
                collectiveCloud +=visibleCloud;
                tempCloud->points = collectiveCloud.points;

                pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
                voxelgrid.setInputCloud (tempCloud);
                voxelgrid.setLeafSize (0.5f, 0.5f, 0.5f);
                voxelgrid.filter(*node->cloud_filtered);

                node->coverage = occlussionCulling->calcCoveragePercent(node->cloud_filtered);
                c = node->coverage - node->parent->coverage;
                extraCovPerViewpointAvg.push_back(c);
                extraCovSum +=c;

                f = node->parent->f_value + ((1/d)*c);
            }
            //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
            else if(heuristicType==SurfaceCoveragewithAccuracyH)
            {
                //accuracy
                double avgAcc = occlussionCulling->calcAvgAccuracy(visibleCloud);
                a = (occlussionCulling->maxAccuracyError - occlussionCulling->calcAvgAccuracy(visibleCloud))/occlussionCulling->maxAccuracyError;
                accuracyPerViewpointAvg.push_back(a);
                accuracySum += avgAcc;

                //coverage
                collectiveCloud.points = node->parent->cloud_filtered->points;
                collectiveCloud +=visibleCloud;
                tempCloud->points = collectiveCloud.points;

                pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
                voxelgrid.setInputCloud (tempCloud);
                voxelgrid.setLeafSize (0.5f, 0.5f, 0.5f);
                voxelgrid.filter(*node->cloud_filtered);

                node->coverage = occlussionCulling->calcCoveragePercent(node->cloud_filtered);
                c = node->coverage - node->parent->coverage;
                extraCovPerViewpointAvg.push_back(c);
                extraCovSum +=c;

                f = node->parent->f_value + ((1/d)*c*a);
            }
            //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//             else if(heuristicType==SurfaceAreaCoverageH)
//             {
//                 //accuracy
//                 double avgAcc = occlussionCulling->calcAvgAccuracy(visibleCloud);
//                 a = (occlussionCulling->maxAccuracyError - occlussionCulling->calcAvgAccuracy(visibleCloud))/occlussionCulling->maxAccuracyError;
//                 accuracyPerViewpointAvg.push_back(a);
//                 accuracySum += avgAcc;
// 
//                 //area
//                 Triangles tempTri, extraTri;
//                 //                meshSurface->meshingPCL(visibleCloud, tempTri, false);
//                 meshSurface->meshingScaleSpaceCGAL(visibleCloud, tempTri, false);
//                 meshSurface->setCGALMeshA(node->parent->surfaceTriangles);
//                 meshSurface->setCGALMeshB(tempTri);
//                 //                node->surfaceTriangles= node->parent->surfaceTriangles;
// 
//                 double extraCovArea = meshSurface->getExtraArea(extraTri);//here the function should increase the number of surfaceTriangles adding (extra triangles)
//                 std::cout<<"reconstructed triangles :"<<tempTri.size()<<std::endl;
//                 std::cout<<"extra triangles :"<<extraTri.size()<<std::endl;
// 
//                 converter to_simple;
//                 node->cloud.points = node->parent->cloud.points;
//                 for(int i=0; i<extraTri.size(); i++)
//                 {
//                     pcl::PointXYZ pt;
//                     Triangle_3 tri = extraTri[i];
//                     for(int j=0; j<3; j++)
//                     {
//                         Point_3 ptCGAL = tri.vertex(j) ;
//                         Point_3_S psimple  = to_simple(ptCGAL);
//                         pt.data[0] = psimple[0];pt.data[1] = psimple[1]; pt.data[2]= psimple[2];
//                         node->cloud.points.push_back(pt);
// 
// 
//                     }
//                 }
// 
//                 meshSurface->meshingScaleSpaceCGAL(node->cloud, node->surfaceTriangles,false);
//                 std::cout<<"new reconstructed triangles :"<<node->surfaceTriangles.size()<<std::endl;
// 
// 
//                 if(debug)
//                 {
//                     std::cout<<"triangles :"<<tempTri.size()<<std::endl;
//                     std::cout<<"after getting extra Tri : "<<node->surfaceTriangles.size()<<std::endl<<std::endl;
//                 }
// 
//                 node->coverage = (meshSurface->calcCGALMeshSurfaceArea(node->surfaceTriangles)/aircraftArea )* 100; //accumelated coverage % instead of the accumelated coverage in terms of the points
// 
//                 extraAreaperViewpointAvg.push_back(extraCovArea);
//                 double AreaCoveragePercent = (extraCovArea/aircraftArea)*100;// extra surface area % used instead of c
//                 extraAreaSum += AreaCoveragePercent;
// 
//                 if(debug)
//                 {
//                     Triangles temp;
//                     double interCovArea = meshSurface->getIntersectionArea(temp);
//                     std::cout<<"node viewpoint area: "<<meshSurface->calcCGALMeshSurfaceArea(tempTri) <<"intersection B:"<<interCovArea<<std::endl;
//                     std::cout<<"Aircraft Area: "<<aircraftArea <<"extra area: "<<extraCovArea<<" extra area percent: "<<AreaCoveragePercent<<std::endl;
//                 }
// 
//                 f = node->parent->f_value + (1/d)*AreaCoveragePercent*a;
// 
//             }
            //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
            else if(heuristicType==VolumetricCoverageH)
            {
                //turning angle
                tf::Quaternion qtParent(node->parent->pose.p.orientation.x,node->parent->pose.p.orientation.y,node->parent->pose.p.orientation.z,node->parent->pose.p.orientation.w);
                tf::Quaternion qtNode(node->pose.p.orientation.x,node->pose.p.orientation.y,node->pose.p.orientation.z,node->pose.p.orientation.w);
                double angle, normAngle;
                angle=qtParent.angleShortestPath(qtNode);
                normAngle=1-angle/(2*M_PI);

                //accuracy
                double avgAcc = occlussionCulling->calcAvgAccuracy(visibleCloud);
                a = (occlussionCulling->maxAccuracyError - occlussionCulling->calcAvgAccuracy(visibleCloud))/occlussionCulling->maxAccuracyError;
                accuracyPerViewpointAvg.push_back(a);
                accuracySum += avgAcc;

                if(debug)
                    std::cout<<"cloud size before accumelation: " <<node->cloud.points.size()<<std::endl;

                // accumelate the cloud
                node->cloud = node->parent->voxels;
                node->cloud += visibleCloud;

                if(debug)
                    std::cout<<"cloud size after accumelation: " <<node->cloud.points.size()<<std::endl;

                //filter the accumelated cloud
                pcl::PointCloud<pcl::PointXYZ>::Ptr nodeCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
                nodeCloudPtr->points = node->cloud.points;
                pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
                voxelgrid.setInputCloud (nodeCloudPtr);
                voxelgrid.setLeafSize (volumetricVoxelRes, volumetricVoxelRes, volumetricVoxelRes);
                voxelgrid.filter(node->voxels);

                //calculate the extra volume or voxels
                double extraVoxelsNum = node->voxels.points.size() - node->parent->voxels.points.size();
                double extraVoxelsRatio = (extraVoxelsNum/(double)modelVoxels.points.size());
                double extraVoxelPercent = (extraVoxelsRatio)*100;

                extraCovPerViewpointAvg.push_back(extraVoxelPercent);
                extraCovSum +=extraVoxelPercent;

                if(debug)
                {
                    std::cout<<"node voxels size after filtering: " <<node->voxels.points.size()<<std::endl;
                    std::cout<<"number of extra voxels: " <<extraVoxelsNum<<std::endl;
                    std::cout<<"extra voxels percent: " <<extraVoxelPercent<<std::endl;
                }

                node->coverage = ((double)node->voxels.points.size()/(double)modelVoxels.points.size()) * 100;

                double normDist = (2.5-d)/2.5;//2.5 = max conn radius

                //heuristic 1
                //                f = node->parent->f_value + (1/d)*extraVoxelPercent*a;

                //heuristic 2
                //                f = node->parent->f_value + (accW*a)+(distW*distNorm)+(covW*extraVoxelsNum);

                //heuristic 3
                f = node->parent->f_value + ( (accW*a)+(distW*normDist)+(angleW*normAngle) )*(extraVoxelsNum);
            }
            //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
            else if(heuristicType==InfoGainVolumetricH)
            {
                // 1 - information gain calculation
                node->totalEntroby = node->parent->totalEntroby;
                node->coveredVoxelsNum = node->parent->coveredVoxelsNum;
                node->coveredVolume = node->parent->coveredVolume;
                node->octree = new octomap::OcTree(*node->parent->octree);
                node->octree->setProbHit(0.9999999);
                node->octree->setProbMiss(0.5);
                node->octree->setClampingThresMax(0.9999999);
                node->octree->setClampingThresMin(0.5);
                pcl::PointCloud<pcl::PointXYZ> visibleCloud;


                for(int i=0; i<node->senPoses.size(); i++)
                {
                    pcl::PointCloud<pcl::PointXYZ> temp;
                    octomap::Pointcloud visibleOctPointCloud;
                    temp = occlussionCulling->extractVisibleSurface(node->senPoses[i].p);
                    visibleCloud+=temp;
                    for(int j = 0;j<temp.points.size();j++)
                    {
                        octomap::point3d endpoint((float) temp.points[j].x,(float) temp.points[j].y,(float) temp.points[j].z);
                        visibleOctPointCloud.push_back(endpoint);
                    }

                    octomap::point3d origin(node->senPoses[i].p.position.x,node->senPoses[i].p.position.y,node->senPoses[i].p.position.z);
                    octomap::KeySet freeKeys,occupiedKeys;
                    node->octree->computeUpdate(visibleOctPointCloud,origin,freeKeys,occupiedKeys,-1);

                    //loop through occupied keys
                    for (octomap::KeySet::iterator it = occupiedKeys.begin(); it != occupiedKeys.end(); ++it) {
                        //calculate accuracy (sensor noise)
                        octomap::point3d center = node->octree->keyToCoord(*it);
                        pcl::PointCloud<pcl::PointXYZ> voxelPoint,transformedVoxel;
                        pcl::PointXYZ pt;
                        pt.x = center.x();pt.y = center.y();pt.z = center.z();
                        voxelPoint.points.push_back(pt);
                        transformedVoxel = occlussionCulling->pointCloudViewportTransform(voxelPoint,node->senPoses[i].p); //newly added
                        double dist = transformedVoxel.points[0].x;
                        //std::cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> distance : "<<dist<<" max depth: "<<maxDepth<<std::endl;
                        if(dist>maxDepth)
                            dist=maxDepth;
                        double normAcc = (occlussionCulling->maxAccuracyError - (0.0000285 * dist * dist*0.5))/occlussionCulling->maxAccuracyError;
                        float lg = octomap::logodds(normAcc);
                        octomap::OcTreeNode* result = node->octree->search (*it);

                        if(result!=NULL)//already occupied
                        {
                            double prob = result->getOccupancy();
//                            double entropy = ( -1*prob*std::log(prob) ) ;//- ( (1-prob)*std::log(1-prob) );
                            double entropy = ( -1*prob*(log(prob)/log(2)) ) ;
                            node->totalEntroby -= entropy;
                            double postProb = (normAcc*prob) / ( (normAcc*prob)+( (1-normAcc)*(1-prob) ) );
                            double logO = octomap::logodds(postProb);
                            node->octree->setNodeValue(*it, logO);
//                            prob = octomap::probability(lg);
//                            entropy = ( -1*prob*std::log(prob) ) ;//- ( (1-prob)*std::log(1-prob) );
                            entropy = ( -1*postProb*(log(postProb)/log(2)) ) ;
                            node->totalEntroby += entropy;
                        }
                        else //NULL voxel
                        {
                            node->coveredVoxelsNum++;
                            double entropy = ( -1*(0.5)*(log(0.5)/log(2)) ) ; //subtract the unknown prob 0.5 entroby
                            node->totalEntroby -= entropy;
                            double postProb = (normAcc*0.5) / ( (normAcc*0.5)+( (1-normAcc)*(1-0.5) ) );
                            double logO = octomap::logodds(postProb);
                            node->octree->setNodeValue(*it, logO);
//                            double prob = octomap::probability(lg);
//                            entropy = ( -1*prob*std::log(prob) ) ;//- ( (1-prob)*std::log(1-prob) );
                            entropy = ( -1*postProb*(log(postProb)/log(2)) ) ;

                            node->coveredVolume += (volumetricVoxelRes*volumetricVoxelRes*volumetricVoxelRes);
                            node->totalEntroby += entropy;
                        }

                    }//end of occupied Keys loop

                    //if one of the mounted sensors did not extract any point cloud, avoid creating nan , -nan values
                    if(temp.points.size()!=0)
                    {
                        double avgAcc = occlussionCulling->calcAvgAccuracy(temp,node->senPoses[i].p);
                        double a = (occlussionCulling->maxAccuracyError - avgAcc)/occlussionCulling->maxAccuracyError;
                        accuracyPerViewpointAvg.push_back(a);
                        accuracySum += avgAcc;
                        //std::cout<<"accuracy per viewpoints: "<<a<<" "<<avgAcc<<std::endl;
                    }

                }//end of sensors loop

                // 2 - coverage for termination
                node->coverage = ((double)node->coveredVolume/modelVolume) * 100;
                double extraVolume = (node->coverage - node->parent->coverage)/100;//to make it normalized
                c = node->coverage - node->parent->coverage;
                extraCovPerViewpointAvg.push_back(c);
                extraCovSum += c;

                // 3- distance
//                double normDist = (maxConnRadius-d)/maxConnRadius;// max conn radius
                double normDist = d/maxConnRadius;// max conn radius

                // 4- turning angle
                tf::Quaternion qtParent(node->parent->pose.p.orientation.x,node->parent->pose.p.orientation.y,node->parent->pose.p.orientation.z,node->parent->pose.p.orientation.w);
                tf::Quaternion qtNode(node->pose.p.orientation.x,node->pose.p.orientation.y,node->pose.p.orientation.z,node->pose.p.orientation.w);
                double angle, normAngle;
                angle=qtParent.angleShortestPath(qtNode);
//                normAngle=1-angle/(2*M_PI);
                normAngle = angle/(2*M_PI);

                // 5- function
                double localE = node->totalEntroby- node->parent->totalEntroby;
//                f = node->totalEntroby + extraVolume + normDist +normAngle;
                f = node->parent->f_value+localE*std::exp(d*-0.2);//*(normAngle);
//                std::cout<<"heuristic value: "<<f<<"local Entropy: "<<localE<<" distance: "<<d<<std::endl;
//                std::cout<<"Total Entroby: "<<node->totalEntroby<<std::endl;
            }
        }
        else //if node is at the same position of the parent with different orientation
        {
            if(heuristicType==SurfaceCoverageH)
            {
                //coverage
                collectiveCloud.points = node->parent->cloud_filtered->points;
                collectiveCloud +=visibleCloud;
                tempCloud->points = collectiveCloud.points;

                pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
                voxelgrid.setInputCloud (tempCloud);
                voxelgrid.setLeafSize (0.5f, 0.5f, 0.5f);
                voxelgrid.filter(*node->cloud_filtered);

                node->coverage = occlussionCulling->calcCoveragePercent(node->cloud_filtered);
                c = node->coverage - node->parent->coverage;
                extraCovPerViewpointAvg.push_back(c);
                extraCovSum +=c;

                f = node->parent->f_value + c;
            }
            //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
            else if(heuristicType==SurfaceCoveragewithOrientationH)
            {
                //turning angle
                tf::Quaternion qtParent(node->parent->pose.p.orientation.x,node->parent->pose.p.orientation.y,node->parent->pose.p.orientation.z,node->parent->pose.p.orientation.w);
                tf::Quaternion qtNode(node->pose.p.orientation.x,node->pose.p.orientation.y,node->pose.p.orientation.z,node->pose.p.orientation.w);
                double angle, normAngle;
                angle=qtParent.angleShortestPath(qtNode);
                normAngle=1-angle/(2*M_PI);

                //coverage
                collectiveCloud.points = node->parent->cloud_filtered->points;
                collectiveCloud +=visibleCloud;
                tempCloud->points = collectiveCloud.points;

                pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
                voxelgrid.setInputCloud (tempCloud);
                voxelgrid.setLeafSize (0.5f, 0.5f, 0.5f);
                voxelgrid.filter(*node->cloud_filtered);

                node->coverage = occlussionCulling->calcCoveragePercent(node->cloud_filtered);
                c = node->coverage - node->parent->coverage;
                extraCovPerViewpointAvg.push_back(c);
                extraCovSum +=c;

                f = node->parent->f_value + normAngle*c;
            }
            //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
            else if(heuristicType==SurfaceCoveragewithAccuracyH)
            {
                //turning angle
                tf::Quaternion qtParent(node->parent->pose.p.orientation.x,node->parent->pose.p.orientation.y,node->parent->pose.p.orientation.z,node->parent->pose.p.orientation.w);
                tf::Quaternion qtNode(node->pose.p.orientation.x,node->pose.p.orientation.y,node->pose.p.orientation.z,node->pose.p.orientation.w);
                double angle, normAngle;
                angle=qtParent.angleShortestPath(qtNode);
                normAngle=1-angle/(2*M_PI);

                //coverage
                collectiveCloud.points = node->parent->cloud_filtered->points;
                collectiveCloud +=visibleCloud;
                tempCloud->points = collectiveCloud.points;

                pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
                voxelgrid.setInputCloud (tempCloud);
                voxelgrid.setLeafSize (0.5f, 0.5f, 0.5f);
                voxelgrid.filter(*node->cloud_filtered);

                node->coverage = occlussionCulling->calcCoveragePercent(node->cloud_filtered);
                c = node->coverage - node->parent->coverage;
                extraCovPerViewpointAvg.push_back(c);
                extraCovSum +=c;

                //accuracy
                double avgAcc = occlussionCulling->calcAvgAccuracy(visibleCloud);
                accuracySum += avgAcc;
                a = (occlussionCulling->maxAccuracyError - occlussionCulling->calcAvgAccuracy(visibleCloud))/occlussionCulling->maxAccuracyError;

                f = node->parent->f_value + a*c*normAngle;

            }
            //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//             else if(heuristicType==SurfaceAreaCoverageH)
//             {
//                 //turning angle
//                 tf::Quaternion qtParent(node->parent->pose.p.orientation.x,node->parent->pose.p.orientation.y,node->parent->pose.p.orientation.z,node->parent->pose.p.orientation.w);
//                 tf::Quaternion qtNode(node->pose.p.orientation.x,node->pose.p.orientation.y,node->pose.p.orientation.z,node->pose.p.orientation.w);
//                 double angle, normAngle;
//                 angle=qtParent.angleShortestPath(qtNode);
//                 normAngle=1-angle/(2*M_PI);
// 
//                 //accuracy
//                 double avgAcc = occlussionCulling->calcAvgAccuracy(visibleCloud);
//                 a = (occlussionCulling->maxAccuracyError - occlussionCulling->calcAvgAccuracy(visibleCloud))/occlussionCulling->maxAccuracyError;
//                 accuracyPerViewpointAvg.push_back(a);
//                 accuracySum += avgAcc;
// 
//                 //surface area
//                 Triangles tempTri, extraTri;
//                 meshSurface->meshingScaleSpaceCGAL(visibleCloud, tempTri,false);
//                 //                meshSurface->meshingPCL(visibleCloud, tempTri,false);
//                 meshSurface->setCGALMeshA(node->parent->surfaceTriangles);
//                 meshSurface->setCGALMeshB(tempTri);
//                 //                node->surfaceTriangles= node->parent->surfaceTriangles;
//                 double extraCovArea = meshSurface->getExtraArea(extraTri);//here the function should increase the number of surfaceTriangles adding (extra triangles)
//                 std::cout<<"reconstructed triangles :"<<tempTri.size()<<std::endl;
//                 std::cout<<"extra triangles :"<<extraTri.size()<<std::endl;
// 
//                 converter to_simple;
//                 node->cloud.points = node->parent->cloud.points;
//                 for(int i=0; i<extraTri.size(); i++)
//                 {
//                     pcl::PointXYZ pt;
//                     Triangle_3 tri = extraTri[i];
//                     for(int j=0; j<3; j++)
//                     {
//                         Point_3 ptCGAL = tri.vertex(j) ;
//                         Point_3_S psimple  = to_simple(ptCGAL);
//                         pt.data[0] = psimple[0];pt.data[1] = psimple[1]; pt.data[2]= psimple[2];
//                         node->cloud.points.push_back(pt);
// 
//                     }
//                 }
//                 meshSurface->meshingScaleSpaceCGAL(node->cloud, node->surfaceTriangles,false);
//                 std::cout<<"new reconstructed triangles :"<<node->surfaceTriangles.size()<<std::endl;
// 
//                 if(debug)
//                 {
//                     std::cout<<"triangles :"<<tempTri.size()<<std::endl;
//                     std::cout<<"after getting extra Tri : "<<node->surfaceTriangles.size()<<std::endl<<std::endl;
//                 }
// 
//                 node->coverage = (meshSurface->calcCGALMeshSurfaceArea(node->surfaceTriangles)/aircraftArea )* 100; //accumelated coverage % instead of the accumelated coverage in terms of the points
// 
//                 extraAreaperViewpointAvg.push_back(extraCovArea);
//                 double AreaCoveragePercent = (extraCovArea/aircraftArea)*100; // extra surface area % used instead of c
//                 extraAreaSum += AreaCoveragePercent;
// 
//                 if(debug)
//                 {
//                     Triangles temp;
//                     double interCovArea = meshSurface->getIntersectionArea(temp);
//                     std::cout<<"node viewpoint area: "<<meshSurface->calcCGALMeshSurfaceArea(tempTri) <<"intersection B:"<<interCovArea<<std::endl;
//                     std::cout<<"Aircraft Area: "<<aircraftArea <<"extra area: "<<extraCovArea<<" extra area percent: "<<AreaCoveragePercent<<std::endl;
//                 }
// 
//                 f = node->parent->f_value + AreaCoveragePercent*normAngle*a;
// 
//             }
            //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
            else if(heuristicType==VolumetricCoverageH)
            {
                //turning angle
                tf::Quaternion qtParent(node->parent->pose.p.orientation.x,node->parent->pose.p.orientation.y,node->parent->pose.p.orientation.z,node->parent->pose.p.orientation.w);
                tf::Quaternion qtNode(node->pose.p.orientation.x,node->pose.p.orientation.y,node->pose.p.orientation.z,node->pose.p.orientation.w);
                double angle, normAngle;
                angle=qtParent.angleShortestPath(qtNode);
                normAngle=1-angle/(2*M_PI);

                //accuracy
                double avgAcc = occlussionCulling->calcAvgAccuracy(visibleCloud);
                a = (occlussionCulling->maxAccuracyError - occlussionCulling->calcAvgAccuracy(visibleCloud))/occlussionCulling->maxAccuracyError;
                accuracyPerViewpointAvg.push_back(a);
                accuracySum += avgAcc;

                // volumetric coverage
                // accumelate the cloud
                if(debug)
                    std::cout<<"cloud size before accumelation: " <<node->cloud.points.size()<<std::endl;

                node->cloud = node->parent->voxels;
                node->cloud += visibleCloud;

                if(debug)
                    std::cout<<"cloud size after accumelation: " <<node->cloud.points.size()<<std::endl;

                //filter the accumelated cloud
                pcl::PointCloud<pcl::PointXYZ>::Ptr nodeCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
                nodeCloudPtr->points = node->cloud.points;
                pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
                voxelgrid.setInputCloud (nodeCloudPtr);
                voxelgrid.setLeafSize (volumetricVoxelRes, volumetricVoxelRes, volumetricVoxelRes);
                voxelgrid.filter(node->voxels);


                //calculate the extra volume or voxels
                double extraVoxelsNum = node->voxels.points.size() - node->parent->voxels.points.size();
                double extraVoxelsRatio = (extraVoxelsNum/(double)modelVoxels.points.size());
                double extraVoxelPercent = (extraVoxelsRatio)*100;

                extraCovPerViewpointAvg.push_back(extraVoxelPercent);
                extraCovSum +=extraVoxelPercent;

                if(debug)
                {
                    std::cout<<"node voxels size after filtering: " <<node->voxels.points.size()<<std::endl;
                    std::cout<<"number of extra voxels: " <<extraVoxelsNum<<std::endl;
                    std::cout<<"extra voxels percent: " <<extraVoxelPercent<<std::endl;
                }

                node->coverage = ((double)node->voxels.points.size()/(double)modelVoxels.points.size()) * 100;

                //heuristic 1
                //                f = node->parent->f_value + normAngle*extraVoxelPercent*a;

                //heuristic 2
                //                f = node->parent->f_value + (accW*a)+(angleW*normAngle)+(covW*extraVoxelsNum);

                //heuristic 3
                f = node->parent->f_value + ( (accW*a)+(angleW*normAngle) )*(extraVoxelsNum);

            }
            //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
            else if(heuristicType==InfoGainVolumetricH)
            {
                // 1 - information gain calculation
                node->totalEntroby = node->parent->totalEntroby;
                node->coveredVoxelsNum = node->parent->coveredVoxelsNum;
                node->coveredVolume = node->parent->coveredVolume;
                node->octree = new octomap::OcTree(*node->parent->octree);
                node->octree->setProbHit(0.9999999);
                node->octree->setProbMiss(0.5);
                node->octree->setClampingThresMax(0.9999999);
                node->octree->setClampingThresMin(0.5);
                pcl::PointCloud<pcl::PointXYZ> visibleCloud;

                for(int i=0; i<node->senPoses.size(); i++)
                {
                    pcl::PointCloud<pcl::PointXYZ> temp;
                    octomap::Pointcloud visibleOctPointCloud;
                    temp = occlussionCulling->extractVisibleSurface(node->senPoses[i].p);
                    visibleCloud+=temp;
                    for(int j = 0;j<temp.points.size();j++)
                    {
                        octomap::point3d endpoint((float) temp.points[j].x,(float) temp.points[j].y,(float) temp.points[j].z);
                        visibleOctPointCloud.push_back(endpoint);
                    }

                    octomap::point3d origin(node->senPoses[i].p.position.x,node->senPoses[i].p.position.y,node->senPoses[i].p.position.z);
                    octomap::KeySet freeKeys,occupiedKeys;
                    node->octree->computeUpdate(visibleOctPointCloud,origin,freeKeys,occupiedKeys,-1);

                    //loop through occupied keys
                    for (octomap::KeySet::iterator it = occupiedKeys.begin(); it != occupiedKeys.end(); ++it) {
                        //calculate accuracy (sensor noise)
                        octomap::point3d center = node->octree->keyToCoord(*it);
                        pcl::PointCloud<pcl::PointXYZ> voxelPoint,transformedVoxel;
                        pcl::PointXYZ pt;
                        pt.x = center.x();pt.y = center.y();pt.z = center.z();
                        voxelPoint.points.push_back(pt);
                        transformedVoxel = occlussionCulling->pointCloudViewportTransform(voxelPoint,node->senPoses[i].p); //newly added
                        double dist = transformedVoxel.points[0].x;
                        //std::cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> distance : "<<dist<<" max depth: "<<maxDepth<<std::endl;

                        if(dist>maxDepth)
                            dist=maxDepth;
                        double normAcc = (occlussionCulling->maxAccuracyError - (0.0000285 * dist * dist*0.5))/occlussionCulling->maxAccuracyError;
                        float lg = octomap::logodds(normAcc);
                        octomap::OcTreeNode* result = node->octree->search (*it);

                        if(result!=NULL)//already occupied
                        {
                            double prob = result->getOccupancy();
//                            double entropy = ( -1*prob*std::log(prob) ) ;//- ( (1-prob)*std::log(1-prob) );
                            double entropy = ( -1*prob*(log(prob)/log(2)) ) ;
                            node->totalEntroby -= entropy;
                            double postProb = (normAcc*prob) / ( (normAcc*prob)+( (1-normAcc)*(1-prob) ) );
                            double logO = octomap::logodds(postProb);
                            node->octree->setNodeValue(*it, logO);
//                            prob = octomap::probability(lg);
//                            entropy = ( -1*prob*std::log(prob) ) ;//- ( (1-prob)*std::log(1-prob) );
                            entropy = ( -1*postProb*(log(postProb)/log(2)) ) ;
                            node->totalEntroby += entropy;
                        }
                        else //NULL
                        {
                            node->coveredVoxelsNum++;
                            double entropy = ( -1*(0.5)*(log(0.5)/log(2)) ) ; //subtract the unknown prob 0.5 entroby
                            node->totalEntroby -= entropy;
                            double postProb = (normAcc*0.5) / ( (normAcc*0.5)+( (1-normAcc)*(1-0.5) ) );
                            double logO = octomap::logodds(postProb);
                            node->octree->setNodeValue(*it, logO);
//                            double prob = octomap::probability(lg);
//                            entropy = ( -1*prob*std::log(prob) ) ;//- ( (1-prob)*std::log(1-prob) );
                            entropy = ( -1*postProb*(log(postProb)/log(2)) ) ;

                            node->coveredVolume += (volumetricVoxelRes*volumetricVoxelRes*volumetricVoxelRes);
                            node->totalEntroby += entropy;
                        }

                    }//end of occupied Keys loop

                    //if one of the mounted sensors did not extract any point cloud, avoid creating nan , -nan values
                    if(temp.points.size()!=0)
                    {
                        double avgAcc = occlussionCulling->calcAvgAccuracy(temp,node->senPoses[i].p);
                        double a = (occlussionCulling->maxAccuracyError - avgAcc)/occlussionCulling->maxAccuracyError;
                        accuracyPerViewpointAvg.push_back(a);
                        accuracySum += avgAcc;
                        //std::cout<<"accuracy per viewpoints: "<<a<<" "<<avgAcc<<std::endl;
                    }

                }//end of sensors loop

                //2 - coverage for termination
                node->coverage = ((double)node->coveredVolume/modelVolume) * 100;
                double extraVolume = (node->coverage - node->parent->coverage)/100;
                c = node->coverage - node->parent->coverage;
                extraCovPerViewpointAvg.push_back(c);
                extraCovSum += c;

                //3 - turning angle
                tf::Quaternion qtParent(node->parent->pose.p.orientation.x,node->parent->pose.p.orientation.y,node->parent->pose.p.orientation.z,node->parent->pose.p.orientation.w);
                tf::Quaternion qtNode(node->pose.p.orientation.x,node->pose.p.orientation.y,node->pose.p.orientation.z,node->pose.p.orientation.w);
                double angle, normAngle;
                angle=qtParent.angleShortestPath(qtNode);
//                normAngle=1-angle/(2*M_PI);
                normAngle = angle/(2*M_PI);

                //4- function
                double localE = node->totalEntroby-node->parent->totalEntroby;
//                f = node->totalEntroby+ extraVolume +normAngle;
                f = node->parent->f_value+localE*std::exp(d*-0.2);//*(normAngle);
//                std::cout<<"heuristic value: "<<f<<"local Entropy: "<<localE<<" distance: "<<d<<std::endl;
//                std::cout<<"Total Entroby: "<<node->totalEntroby<<std::endl;
            }
        }
        node->f_value  = f;
        if(debug)
        {
            std::cout<<"\nchild collective cloud after filtering size: "<<node->cloud_filtered->size()<<"\n";
            std::cout<<"parent coverage :"<<node->parent->coverage<<" current node coverage: "<<node->coverage<<"\n";
            std::cout<<"extra coverage c : "<<c<<"\n";
            std::cout<<"parent f value calculation: "<<f<<"\n";
        }

    }else //if the node is root
    {
//        double avgAcc = occlussionCulling->calcAvgAccuracy(visibleCloud);
//        double a = (occlussionCulling->maxAccuracyError - occlussionCulling->calcAvgAccuracy(visibleCloud))/occlussionCulling->maxAccuracyError;
//        accuracyPerViewpointAvg.push_back(a);
//        accuracySum += avgAcc;
        //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//         if(heuristicType==SurfaceAreaCoverageH)
//         {
//             meshSurface->meshingScaleSpaceCGAL(visibleCloud, node->surfaceTriangles,false);
//             node->coverage = (meshSurface->calcCGALMeshSurfaceArea(node->surfaceTriangles)/aircraftArea )* 100; //accumelated coverage % instead of the accumelated coverage in terms of the points
//             node->cloud.points = visibleCloud.points;
//         }else
        //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	if(heuristicType==VolumetricCoverageH)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr nodeCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
            nodeCloudPtr->points = visibleCloud.points;
            pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
            voxelgrid.setInputCloud (nodeCloudPtr);
            voxelgrid.setLeafSize (volumetricVoxelRes, volumetricVoxelRes, volumetricVoxelRes);
            voxelgrid.filter(node->voxels);

            node->coverage = ((double)node->voxels.points.size()/(double)modelVoxels.points.size()) * 100;

            extraCovPerViewpointAvg.push_back(node->coverage);
            extraCovSum +=node->coverage;

            node->cloud.points = visibleCloud.points;
        }
        //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        else if(heuristicType==InfoGainVolumetricH)
        {
            // 1 - information gain for root
            node->octree = new octomap::OcTree(volumetricVoxelRes);
            node->octree->setProbHit(0.9999999);
            node->octree->setProbMiss(0.5);
            node->octree->setClampingThresMax(0.9999999);
            node->octree->setClampingThresMin(0.5);
            node->totalEntroby = modelTotalEntroby;
            pcl::PointCloud<pcl::PointXYZ> visibleCloud;

            for(int i=0; i<node->senPoses.size(); i++)
            {
                pcl::PointCloud<pcl::PointXYZ> temp;
                octomap::Pointcloud visibleOctPointCloud;
                temp = occlussionCulling->extractVisibleSurface(node->senPoses[i].p);
                visibleCloud+=temp;
                for(int j = 0;j<temp.points.size();j++)
                {
                    octomap::point3d endpoint((float) temp.points[j].x,(float) temp.points[j].y,(float) temp.points[j].z);
                    visibleOctPointCloud.push_back(endpoint);
                    node->coveredCloud.push_back(endpoint);
                }

                octomap::point3d origin(node->senPoses[i].p.position.x,node->senPoses[i].p.position.y,node->senPoses[i].p.position.z);
                octomap::KeySet freeKeys,occupiedKeys;
                node->octree->computeUpdate(visibleOctPointCloud,origin,freeKeys,occupiedKeys,-1);

                //loop through occupied keys
                for (octomap::KeySet::iterator it = occupiedKeys.begin(); it != occupiedKeys.end(); ++it) {
                    //calculate accuracy (sensor noise)
                    octomap::point3d center =node->octree->keyToCoord(*it);
                    pcl::PointCloud<pcl::PointXYZ> voxelPoint,transformedVoxel;
                    pcl::PointXYZ pt;
                    pt.x = center.x();pt.y = center.y();pt.z = center.z();
                    voxelPoint.points.push_back(pt);
                    transformedVoxel = occlussionCulling->pointCloudViewportTransform(voxelPoint,node->senPoses[i].p); //newly added
                    double dist = transformedVoxel.points[0].x;
                    //std::cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> distance : "<<dist<<" max depth: "<<maxDepth<<std::endl;

                    if(dist>maxDepth)
                        dist=maxDepth;
                    double normAcc = (occlussionCulling->maxAccuracyError - (0.0000285 * dist * dist*0.5))/occlussionCulling->maxAccuracyError;
                    float lg = octomap::logodds(normAcc);
                    octomap::OcTreeNode* result = node->octree->search(*it);

                    if(result!=NULL)//already occupied
                    {
                        double prob = result->getOccupancy();
//                            double entropy = ( -1*prob*std::log(prob) ) ;//- ( (1-prob)*std::log(1-prob) );
                        double entropy = ( -1*prob*(log(prob)/log(2)) ) ;
                        node->totalEntroby -= entropy;
                        double postProb = (normAcc*prob) / ( (normAcc*prob)+( (1-normAcc)*(1-prob) ) );
                        double logO = octomap::logodds(postProb);
                        node->octree->setNodeValue(*it, logO);
//                            prob = octomap::probability(lg);
//                            entropy = ( -1*prob*std::log(prob) ) ;//- ( (1-prob)*std::log(1-prob) );
                        entropy = ( -1*postProb*(log(postProb)/log(2)) ) ;
                        node->totalEntroby += entropy;

                    }else{ // NULL
                        node->coveredVoxelsNum++;
                        double entropy = ( -1*(0.5)*(log(0.5)/log(2)) ) ; //subtract the unknown prob 0.5 entroby
                        node->totalEntroby -= entropy;
                        double postProb = (normAcc*0.5) / ( (normAcc*0.5)+( (1-normAcc)*(1-0.5) ) );
                        double logO = octomap::logodds(postProb);
                        node->octree->setNodeValue(*it, logO);
//                            double prob = octomap::probability(lg);
//                            entropy = ( -1*prob*std::log(prob) ) ;//- ( (1-prob)*std::log(1-prob) );
                        entropy = ( -1*postProb*(log(postProb)/log(2)) ) ;

                        node->coveredVolume += (volumetricVoxelRes*volumetricVoxelRes*volumetricVoxelRes);
                        node->totalEntroby += entropy;
                    }

                }//end of occupied Keys loop

                //if one of the mounted sensors did not extract any point cloud, avoid creating nan , -nan values
                if(temp.points.size()!=0)
                {
                    double avgAcc = occlussionCulling->calcAvgAccuracy(temp,node->senPoses[i].p);
                    double a = (occlussionCulling->maxAccuracyError - avgAcc)/occlussionCulling->maxAccuracyError;
                    accuracyPerViewpointAvg.push_back(a);
                    accuracySum += avgAcc;
                    //std::cout<<"accuracy per viewpoints: "<<a<<" "<<avgAcc<<std::endl;
                }


            }//end of sensors loop

            //2- coverage
            node->coverage = (double)node->coveredVolume/modelVolume * 100;
            double c = node->coverage;
            extraCovPerViewpointAvg.push_back(c);
            extraCovSum += c;
            std::cout<<"Model Entropy :"<<modelTotalEntroby<<std::endl;
            node->f_value = node->totalEntroby;

        }
        else {
            //coverage
            tempCloud->points = visibleCloud.points;

            pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
            voxelgrid.setInputCloud (tempCloud);
            voxelgrid.setLeafSize (0.5f, 0.5f, 0.5f);
            voxelgrid.filter(*node->cloud_filtered);

            node->coverage = occlussionCulling->calcCoveragePercent(node->cloud_filtered);

            extraCovPerViewpointAvg.push_back(node->coverage);
            extraCovSum += node->coverage;
        }

        if(heuristicType != InfoGainVolumetricH)
        {
            node->f_value = 0;//root node
            node->g_value = 0;//root node
        }
    }

    if(debug)
        std::cout<<"<<<<<<finished node heuristic calculation>>>>>>"<<std::endl;
}
void CoveragePathPlanningHeuristic::displayGradualProgress(Node *node)
{
    //########display the covered points##########
    sensor_msgs::PointCloud2 cloud1;
    pcl::toROSMsg(*(node->cloud_filtered), cloud1);
    cloud1.header.frame_id = "map";
    cloud1.header.stamp = ros::Time::now();
    coveredPointsPub.publish(cloud1);

    //########display the octomap##########
    if(heuristicType == InfoGainVolumetricH)
    {
        octomap_msgs::Octomap octomap ;
        octomap.binary = 1 ;
        octomap.id = 1 ;
        octomap.resolution =0.25;
        octomap.header.frame_id = "map";
        octomap.header.stamp = ros::Time::now();
        bool res = octomap_msgs::fullMapToMsg(*node->octree, octomap);
        if(res)
        {
            octomapPub.publish(octomap);
        }
        else
        {
            ROS_WARN("OCT Map serialization failed!");
        }
    }

    //########display the point selected##########
    std::vector<geometry_msgs::Point> points;
    geometry_msgs::PoseArray selectedPoses;
    geometry_msgs::PoseArray selectedSensorPoses;
    geometry_msgs::Point linept;
    linept.x = node->pose.p.position.x; linept.y = node->pose.p.position.y; linept.z = node->pose.p.position.z;
    points.push_back(linept);
    //    visualization_msgs::Marker pointsList = drawPoints(points,10,1,10000,0.05);
    visualization_msgs::Marker pointsList = drawPoints(points,3,10000);
    pathPointPub.publish(pointsList);
    selectedPoses.poses.push_back(node->pose.p);
    for(int i =0; i<node->senPoses.size();i++)
    selectedSensorPoses.poses.push_back(node->senPoses[i].p);

    selectedSensorPoses.header.frame_id= "map";
    selectedSensorPoses.header.stamp = ros::Time::now();
    sensorPosesPub.publish(selectedSensorPoses);

    selectedPoses.header.frame_id= "map";
    selectedPoses.header.stamp = ros::Time::now();
    selectedPosePub.publish(selectedPoses);
    selectedPoses.poses.erase(selectedPoses.poses.begin(), selectedPoses.poses.end());
    selectedSensorPoses.poses.erase(selectedSensorPoses.poses.begin(), selectedSensorPoses.poses.end());


    int coveI = (int)node->coverage;

    //########display FOV##########
    //        if (coveI != 0 && %10==0)
//    {
        for(int i =0; i<node->senPoses.size(); i++)
            occlussionCulling->visualizeFOV(node->senPoses[i].p);
//    }

    //########display the path every 1% coverage########
    if (debug == true)
        std::cout<<"\n\n\n\n**********************COVERAGE delta:" <<coveI<<"\n\n\n\n";
    if ( coveI%1==0)
    {
        if (debug == true)
            std::cout<<"INSIDE PUBLISHING"<<"\n";


        //  publish path and pring the path each 1%
        ofstream path_file;
        std::string path = ros::package::getPath("sspp");
        std::string fileloc = path+ "/resources/path_testfile.txt";
        path_file.open(fileloc.c_str());
        std::vector<geometry_msgs::Point> lines;
        geometry_msgs::Point linepoint;
        Node *test_path;
        test_path = node;
        double yaw;
        while (test_path != NULL)
        {
            tf::Quaternion qt(test_path->pose.p.orientation.x,test_path->pose.p.orientation.y,test_path->pose.p.orientation.z,test_path->pose.p.orientation.w);
            yaw = tf::getYaw(qt);
            path_file << test_path->pose.p.position.x<<" "<<test_path->pose.p.position.y<<" "<<test_path->pose.p.position.z<<" "<<yaw<<"\n";
            if (test_path->parent != NULL)
            {
                linepoint.x = test_path->pose.p.position.x; linepoint.y = test_path->pose.p.position.y; linepoint.z = test_path->pose.p.position.z;
                lines.push_back(linepoint);
                linepoint.x = test_path->parent->pose.p.position.x; linepoint.y = test_path->parent->pose.p.position.y; linepoint.z = test_path->parent->pose.p.position.z;
                lines.push_back(linepoint);
            }
            test_path = test_path->parent;
        }

        path_file.close();
        visualization_msgs::Marker linesList = drawLines(lines,333333,1,1000000,0.2);
        pathPub.publish(linesList);
    }
}

void CoveragePathPlanningHeuristic::loadOBJFile(const char* filename, std::vector<Eigen::Vector3f>& points, std::list<CGALTriangle>& triangles)
{
    FILE* file = fopen(filename, "rb");
    if(!file)
    {
        std::cerr << "file not exist" << std::endl;
        return;
    }

    bool has_normal = false;
    bool has_texture = false;
    char line_buffer[2000];
    while(fgets(line_buffer, 2000, file))
    {
        char* first_token = strtok(line_buffer, "\r\n\t ");
        if(!first_token || first_token[0] == '#' || first_token[0] == 0)
            continue;

        switch(first_token[0])
        {
        case 'v':
        {
            if(first_token[1] == 'n')
            {
                strtok(NULL, "\t ");
                strtok(NULL, "\t ");
                strtok(NULL, "\t ");
                has_normal = true;
            }
            else if(first_token[1] == 't')
            {
                strtok(NULL, "\t ");
                strtok(NULL, "\t ");
                has_texture = true;
            }
            else
            {
                float x = (float)atof(strtok(NULL, "\t "));
                float y = (float)atof(strtok(NULL, "\t "));
                float z = (float)atof(strtok(NULL, "\t "));
                Eigen::Vector3f p(x, y, z);
                points.push_back(p);
            }
        }
            break;
        case 'f':
        {
            CGALTriangle tri;
            char* data[30];
            int n = 0;
            while((data[n] = strtok(NULL, "\t \r\n")) != NULL)
            {
                if(strlen(data[n]))
                    n++;
            }

            for(int t = 0; t < (n - 2); ++t)
            {
                if((!has_texture) && (!has_normal))
                {
                    Point p1(points[atoi(data[0]) - 1][0],points[atoi(data[0]) - 1][1],points[atoi(data[0]) - 1][2]);
                    Point p2(points[atoi(data[1]) - 1][0],points[atoi(data[1]) - 1][1],points[atoi(data[1]) - 1][2]);
                    Point p3(points[atoi(data[2]) - 1][0],points[atoi(data[2]) - 1][1],points[atoi(data[2]) - 1][2]);
                    tri = CGALTriangle(p1,p2,p3);
                    if(!CGAL::collinear(p1,p2,p3))
                    {
                        triangles.push_back(tri);
                    }
                }
                else
                {
                    const char *v1;
                    uint indxs[3];
                    for(int i = 0; i < 3; i++)
                    {
                        // vertex ID
                        if(i == 0)
                            v1 = data[0];
                        else
                            v1 = data[t + i];

                        indxs[i] = atoi(v1) - 1;
                    }
                    Point p1(points[indxs[0]][0],points[indxs[0]][1],points[indxs[0]][2]);
                    Point p2(points[indxs[1]][0],points[indxs[1]][1],points[indxs[1]][2]);
                    Point p3(points[indxs[2]][0],points[indxs[2]][1],points[indxs[2]][2]);
                    tri = CGALTriangle(p1,p2,p3);
                    if(!CGAL::collinear(p1,p2,p3))
                    {
                        triangles.push_back(tri);
                    }
                }
            }
        }
        }
    }
}

}
