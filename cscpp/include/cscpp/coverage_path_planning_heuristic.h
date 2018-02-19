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
#ifndef CPP_HEURISTICT_H_
#define CPP_HEURISTICT_H_
#include <stdio.h>
#include <stdlib.h>
#include <cscpp/occlusion_culling_gpu.h>
#include <cscpp/occlusion_culling.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Segment_3.h>
#include <CGAL/AABB_triangle_primitive.h>
#include "fcl/math/vec_3f.h"
#include "fcl/math/transform.h"
#include "sspp/ssppexception.h"
#include "sspp/node.h"
#include "sspp/heuristic_interface.h"
#include "sspp/rviz_drawing_tools.h"
// #include "component_test/mesh_surface.h"
#include <octomap_msgs/Octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
using namespace std;

typedef CGAL::Simple_cartesian<double> K;
typedef K::FT FT;
typedef K::Ray_3 Ray;
typedef K::Line_3 Line1;
typedef K::Point_3 Point;
typedef K::Segment_3 Segment;
typedef K::Triangle_3 CGALTriangle;
typedef std::list<CGALTriangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree1;
typedef CGAL::Cartesian_converter<K,exactKernel > SimpleToExactConverter;

enum{SurfaceCoverageH,SurfaceCoveragewithOrientationH,SurfaceCoveragewithAccuracyH,SurfaceAreaCoverageH,VolumetricCoverageH,InfoGainVolumetricH};

namespace SSPP
{

class CoveragePathPlanningHeuristic:public Heuristic
{
public:
    CoveragePathPlanningHeuristic(ros::NodeHandle & nh, std::string collisionCheckModelP, std::string occlusionCullingModelN, bool d,bool gradualV, int hType);
    ~CoveragePathPlanningHeuristic();
    bool isCost();
    void setCoverageTarget(double coverageTarget);
    void setCoverageTolerance(double coverageTolerance);
    void setDebug(bool debug);
    void setMaxMinSensorAccuracy(std::vector<geometry_msgs::PoseArray> sensorsViewsSS);
    void calculateHeuristic(Node *n);
    bool terminateConditionReached(Node *node);
    bool isConnectionConditionSatisfied(SearchSpaceNode *temp, SearchSpaceNode *S);
    bool isFilteringConditionSatisfied(geometry_msgs::Pose pose, geometry_msgs::PoseArray &correspondingSensorPoses, double minDist, double maxDist, pcl::PointCloud<pcl::PointXYZ>& globalCloud, std::vector<pcl::PointCloud<pcl::PointXYZ> >& accuracyClusters, double accuracyThreshhold);
    void displayProgress(vector<Tree> tree);
    void displayGradualProgress(Node *node);
    double pointCloudDiff(pcl::PointCloud<pcl::PointXYZ>::Ptr globalCloudPtr, pcl::PointCloud<pcl::PointXYZ>::Ptr& pointCloudDiffPtr );
    void clusteringPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZ> >& clustersPointCloudVec, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudDiffPtr);
    void findClusterOuterPoints(geometry_msgs::PoseArray waypoints, pcl::PointCloud<pcl::PointXYZ>& cloudHull);
    void findClusterBB(pcl::PointCloud<pcl::PointXYZ> clusterPoints, geometry_msgs::Vector3 &gridSize, geometry_msgs::Pose &gridStart);
    int getHeuristicType();
private:
    void loadOBJFile(const char* filename, std::vector<fcl::Vec3f>& points, std::list<CGALTriangle>& triangles);
    OcclusionCullingGPU* occlussionCulling;
//     MeshSurface* meshSurface;
    pcl::VoxelGridOcclusionEstimationGPU originalCloudFilteredVoxels;
    bool debug;
    bool gradualVisualization;
    int heuristicType;
    double coverageTarget;
    double modelVolume;
    double coverageTolerance;
    double volumetricVoxelRes;
    double voxelResForConn;
    double maxConnRadius;
    double maxDepth;
    double accW, distW, covW, angleW;
    std::vector<double> accuracyPerViewpointAvg, extraCovPerViewpointAvg, extraAreaperViewpointAvg;
    double accuracySum, extraCovSum, extraAreaSum, aircraftArea;
    double modelTotalEntroby;
    int selectedPointsNum,occupiedVoxelsNum;
    ros::Publisher treePub;
    ros::Publisher coveredPointsPub;
    ros::Publisher pathPointPub;
    ros::Publisher pathPub;
    ros::Publisher octomapPub;
    ros::Publisher hullPub;
    ros::Publisher selectedPosePub;
    ros::Publisher sensorPosesPub;
    std::list<CGALTriangle> triangles;
    std::vector<fcl::Vec3f> modelPoints;
    pcl::PointCloud<pcl::PointXYZ> modelVoxels;
    pcl::PointCloud<pcl::PointXYZ> modelVoxelsForConn;
    Tree1* cgalTree;

};

}
#endif /*CPP_HEURISTICT_H_*/
