/***************************************************************************
 *   Copyright (C) 2015 - 2018 by                                          *
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

#include "cscpp/occlusion_culling.h"


OcclusionCulling::OcclusionCulling(ros::NodeHandle &n, std::string modelName):
    nh(n),
    model(modelName),
    fc(true)
{
//   original_pub = nh.advertise<sensor_msgs::PointCloud2>("original_point_cloud", 10);
//   visible_pub = nh.advertise<sensor_msgs::PointCloud2>("occlusion_free_cloud", 100);

   fov_pub = n.advertise<visualization_msgs::MarkerArray>("fov", 10);
   cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
   cloudCopy = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
   filtered_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);

   occlusionFreeCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
   FrustumCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
   std::string path = ros::package::getPath("component_test");
   pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/"+model, *cloud);
   cloudCopy->points = cloud->points;
   voxelRes = 0.5;
   OriginalVoxelsSize=0.0;
   id=0.0;
   voxelFilterOriginal.setInputCloud (cloud);
   voxelFilterOriginal.setLeafSize (voxelRes, voxelRes, voxelRes);
   voxelFilterOriginal.initializeVoxelGrid();
   min_b1 = voxelFilterOriginal.getMinBoxCoordinates ();
   max_b1 = voxelFilterOriginal.getMaxBoxCoordinates ();
   for (int kk = min_b1.z (); kk <= max_b1.z (); ++kk)
   {
       for (int jj = min_b1.y (); jj <= max_b1.y (); ++jj)
       {
           for (int ii = min_b1.x (); ii <= max_b1.x (); ++ii)
           {
               Eigen::Vector3i ijk1 (ii, jj, kk);
               int index1 = voxelFilterOriginal.getCentroidIndexAt (ijk1);
               if(index1!=-1)
               {
                   OriginalVoxelsSize++;
               }

           }
       }
   }

   pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
   voxelgrid.setInputCloud (cloud);
   voxelgrid.setLeafSize (0.5f, 0.5f, 0.5f);
   voxelgrid.filter (*filtered_cloud);

   fc.setInputCloud (cloud);
   fc.setVerticalFOV (45);
   fc.setHorizontalFOV (58);
   fc.setNearPlaneDistance (0.7);
   fc.setFarPlaneDistance (6.0);

}

OcclusionCulling::OcclusionCulling(ros::NodeHandle &n, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPtr):
    nh(n),
    fc(true)
{
//   original_pub = nh.advertise<sensor_msgs::PointCloud2>("original_point_cloud", 10);
//   visible_pub = nh.advertise<sensor_msgs::PointCloud2>("occlusion_free_cloud", 100);

   fov_pub = nh.advertise<visualization_msgs::MarkerArray>("fov", 100);
   cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
   cloudCopy = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
   filtered_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);

   occlusionFreeCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
   FrustumCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
   cloud->points = cloudPtr->points;
   cloudCopy->points = cloud->points;
    
   voxelRes = 0.1;
   frame_id = "world";
   OriginalVoxelsSize=0.0;
   id=0.0;
   voxelFilterOriginal.setInputCloud (cloud);
   voxelFilterOriginal.setLeafSize (voxelRes, voxelRes, voxelRes);
   voxelFilterOriginal.initializeVoxelGrid();
   min_b1 = voxelFilterOriginal.getMinBoxCoordinates ();
   max_b1 = voxelFilterOriginal.getMaxBoxCoordinates ();
   for (int kk = min_b1.z (); kk <= max_b1.z (); ++kk)
   {
       for (int jj = min_b1.y (); jj <= max_b1.y (); ++jj)
       {
           for (int ii = min_b1.x (); ii <= max_b1.x (); ++ii)
           {
               Eigen::Vector3i ijk1 (ii, jj, kk);
               int index1 = voxelFilterOriginal.getCentroidIndexAt (ijk1);
               if(index1!=-1)
               {
                   OriginalVoxelsSize++;
               }

           }
       }
   }

   pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
   voxelgrid.setInputCloud (cloud);
   voxelgrid.setLeafSize (0.1f, 0.1f, 0.1f);
   voxelgrid.filter (*filtered_cloud);

   fc.setInputCloud (cloud);
   fc.setVerticalFOV (45);
   fc.setHorizontalFOV (60);
   fc.setNearPlaneDistance (0.5);
   fc.setFarPlaneDistance (8.0);

}

OcclusionCulling::OcclusionCulling(std::string modelName):
    model(modelName),
    fc(true)
{
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
    filtered_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
    cloudCopy = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
    FrustumCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);

//    occlusionFreeCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/"+model, *cloud);
    cloudCopy->points = cloud->points;
    voxelRes = 0.5;
    OriginalVoxelsSize=0.0;
    id=0.0;
    voxelFilterOriginal.setInputCloud (cloud);
    voxelFilterOriginal.setLeafSize (voxelRes, voxelRes, voxelRes);
    voxelFilterOriginal.initializeVoxelGrid();
    min_b1 = voxelFilterOriginal.getMinBoxCoordinates ();
    max_b1 = voxelFilterOriginal.getMaxBoxCoordinates ();
    for (int kk = min_b1.z (); kk <= max_b1.z (); ++kk)
    {
        for (int jj = min_b1.y (); jj <= max_b1.y (); ++jj)
        {
            for (int ii = min_b1.x (); ii <= max_b1.x (); ++ii)
            {
                Eigen::Vector3i ijk1 (ii, jj, kk);
                int index1 = voxelFilterOriginal.getCentroidIndexAt (ijk1);
                if(index1!=-1)
                {
                    OriginalVoxelsSize++;
                }

            }
        }
    }
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setInputCloud (cloud);
    voxelgrid.setLeafSize (0.5f, 0.5f, 0.5f);
    voxelgrid.filter (*filtered_cloud);

    fc.setInputCloud (cloud);
    fc.setVerticalFOV (45);
    fc.setHorizontalFOV (58);
    fc.setNearPlaneDistance (0.7);
    fc.setFarPlaneDistance (6.0);

}



OcclusionCulling::OcclusionCulling():
    model(NULL),
    fc(true)
{
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
    filtered_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
    cloudCopy = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);    
    FrustumCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);

//    occlusionFreeCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/scaled_desktop.pcd", *cloud);
    cloudCopy->points = cloud->points;
    voxelRes = 0.5;
    OriginalVoxelsSize=0.0;
    id=0.0;
    voxelFilterOriginal.setInputCloud (cloud);
    voxelFilterOriginal.setLeafSize (voxelRes, voxelRes, voxelRes);
    voxelFilterOriginal.initializeVoxelGrid();
    min_b1 = voxelFilterOriginal.getMinBoxCoordinates ();
    max_b1 = voxelFilterOriginal.getMaxBoxCoordinates ();
    for (int kk = min_b1.z (); kk <= max_b1.z (); ++kk)
    {
        for (int jj = min_b1.y (); jj <= max_b1.y (); ++jj)
        {
            for (int ii = min_b1.x (); ii <= max_b1.x (); ++ii)
            {
                Eigen::Vector3i ijk1 (ii, jj, kk);
                int index1 = voxelFilterOriginal.getCentroidIndexAt (ijk1);
                if(index1!=-1)
                {
                    OriginalVoxelsSize++;
                }

            }
        }
    }
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setInputCloud (cloud);
    voxelgrid.setLeafSize (0.5f, 0.5f, 0.5f);
    voxelgrid.filter (*filtered_cloud);

    fc.setInputCloud (cloud);
    fc.setVerticalFOV (45);
    fc.setHorizontalFOV (58);
    fc.setNearPlaneDistance (0.7);
    fc.setFarPlaneDistance (6.0);

}
OcclusionCulling::~OcclusionCulling()
{
}

bool OcclusionCulling::contains(pcl::PointCloud<pcl::PointXYZ> c, pcl::PointXYZ p) {
    pcl::PointCloud<pcl::PointXYZ>::iterator it = c.begin();
    for (; it != c.end(); ++it) {
        if (it->x == p.x && it->y == p.y && it->z == p.z)
            return true;
    }
    return false;
}

//c2 the cloud that you want to compare with the original cloud in the occlusion culling
pcl::PointCloud<pcl::PointXYZ> OcclusionCulling::pointsDifference(pcl::PointCloud<pcl::PointXYZ> c2) {
    pcl::PointCloud<pcl::PointXYZ> inter;
    pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin();
    for (; it != cloud->end(); ++it) {
        if (!contains(c2, *it))
            inter.push_back(*it);
    }

    return inter;
}

pcl::PointCloud<pcl::PointXYZ> OcclusionCulling::extractVisibleSurface(geometry_msgs::Pose location)
{
    // 1 *****Frustum Culling*******
    pcl::PointCloud <pcl::PointXYZ>::Ptr output (new pcl::PointCloud <pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr occlusionFreeCloud_local(new pcl::PointCloud<pcl::PointXYZ>);

//    pcl::FrustumCullingTT fc (true);
//    fc.setInputCloud (cloud);
//    fc.setVerticalFOV (45);
//    fc.setHorizontalFOV (58);
//    fc.setNearPlaneDistance (0.7);
//    fc.setFarPlaneDistance (6.0);


    Eigen::Matrix4f camera_pose;
    Eigen::Matrix3d Rd;
    Eigen::Matrix3f Rf;

    camera_pose.setZero ();

    tf::Quaternion qt;
    qt.setX(location.orientation.x);
    qt.setY(location.orientation.y);
    qt.setZ(location.orientation.z);
    qt.setW(location.orientation.w);
    tf::Matrix3x3 R_tf(qt);
    tf::matrixTFToEigen(R_tf,Rd);
    Rf = Rd.cast<float>();
    camera_pose.block (0, 0, 3, 3) = Rf;
    Eigen::Vector3f T;
    T (0) = location.position.x; T (1) = location.position.y; T (2) = location.position.z;
    camera_pose.block (0, 3, 3, 1) = T;
    camera_pose (3, 3) = 1;
    fc.setCameraPose (camera_pose);
    ros::Time tic = ros::Time::now();
    fc.filter (*output);
    ros::Time toc = ros::Time::now();

//    std::cout<<"\nFrustum Filter took:"<< toc.toSec() - tic.toSec();

    FrustumCloud->points= output->points;

    //2:****voxel grid occlusion estimation *****
    Eigen::Quaternionf quat(qt.w(),qt.x(),qt.y(),qt.z());
    output->sensor_origin_  = Eigen::Vector4f(T[0],T[1],T[2],0);
    output->sensor_orientation_= quat;
    pcl::VoxelGridOcclusionEstimationT voxelFilter;
    voxelFilter.setInputCloud (output);
    //voxelFilter.setLeafSize (0.03279f, 0.03279f, 0.03279f);
    voxelFilter.setLeafSize (voxelRes, voxelRes, voxelRes);
    voxelFilter.initializeVoxelGrid();

    int state,ret;

    pcl::PointXYZ pt,p1,p2;
    pcl::PointXYZRGB point;
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > out_ray;
    std::vector<geometry_msgs::Point> lineSegments;
    geometry_msgs::Point linePoint;

    // iterate over the entire frustum points
    for ( int i = 0; i < (int)output->points.size(); i ++ )
    {
        pcl::PointXYZ ptest = output->points[i];
        Eigen::Vector3i ijk = voxelFilter.getGridCoordinates( ptest.x, ptest.y, ptest.z);
        // process all free voxels
        int index = voxelFilter.getCentroidIndexAt (ijk);
        Eigen::Vector4f centroid = voxelFilter.getCentroidCoordinate (ijk);
        point = pcl::PointXYZRGB(0,244,0);
        point.x = centroid[0];
        point.y = centroid[1];
        point.z = centroid[2];

        if(index!=-1 )
        {
            out_ray.clear();
            ret = voxelFilter.occlusionEstimation( state,out_ray, ijk);
            //                        std::cout<<"State is:"<<state<<"\n";

            if(state != 1)
            {
                // estimate direction to target voxel
                Eigen::Vector4f direction = centroid - cloud->sensor_origin_;
                direction.normalize ();
                // estimate entry point into the voxel grid
                float tmin = voxelFilter.rayBoxIntersection (cloud->sensor_origin_, direction,p1,p2);
                if(tmin!=-1)
                {
                    // coordinate of the boundary of the voxel grid
                    Eigen::Vector4f start = cloud->sensor_origin_ + tmin * direction;
                    linePoint.x = cloud->sensor_origin_[0]; linePoint.y = cloud->sensor_origin_[1]; linePoint.z = cloud->sensor_origin_[2];
                    //std::cout<<"Box Min X:"<<linePoint.x<<" y:"<< linePoint.y<<" z:"<< linePoint.z<<"\n";
                    lineSegments.push_back(linePoint);

                    linePoint.x = start[0]; linePoint.y = start[1]; linePoint.z = start[2];
                    //std::cout<<"Box Max X:"<<linePoint.x<<" y:"<< linePoint.y<<" z:"<< linePoint.z<<"\n";
                    lineSegments.push_back(linePoint);

                    linePoint.x = start[0]; linePoint.y = start[1]; linePoint.z = start[2];
                    //std::cout<<"Box Max X:"<<linePoint.x<<" y:"<< linePoint.y<<" z:"<< linePoint.z<<"\n";
                    lineSegments.push_back(linePoint);

                    linePoint.x = centroid[0]; linePoint.y = centroid[1]; linePoint.z = centroid[2];
                    //std::cout<<"Box Max X:"<<linePoint.x<<" y:"<< linePoint.y<<" z:"<< linePoint.z<<"\n";
                    lineSegments.push_back(linePoint);

                    occlusionFreeCloud_local->points.push_back(ptest);
                    occlusionFreeCloud->points.push_back(ptest);

                }
            }
        }


    }
    freeCloud.points = occlusionFreeCloud_local->points;
    return freeCloud;
}
float OcclusionCulling::calcCoveragePercent(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{

    // *******************original cloud Grid***************************
    //used VoxelGridOcclusionEstimationT since the voxelGrid does not include getcentroid function
//        pcl::VoxelGridOcclusionEstimationT voxelFilterOriginal;
//        voxelFilterOriginal.setInputCloud (cloud);
//        voxelFilterOriginal.setLeafSize (voxelRes, voxelRes, voxelRes);
//        voxelFilterOriginal.initializeVoxelGrid();

     //*******************Occupied Cloud Grid***************************
    ros::Time covpercent_begin = ros::Time::now();
    pcl::VoxelGridOcclusionEstimationT voxelFilterOccupied;
//        voxelFilterOccupied.setInputCloud (occlusionFreeCloud);
    voxelFilterOccupied.setInputCloud (cloud_filtered);
    voxelFilterOccupied.setLeafSize (voxelRes, voxelRes, voxelRes);
    voxelFilterOccupied.initializeVoxelGrid();



     //*****************************************************************
    Eigen::Vector3i  min_b = voxelFilterOccupied.getMinBoxCoordinates ();
    Eigen::Vector3i  max_b = voxelFilterOccupied.getMaxBoxCoordinates ();
//        Eigen::Vector3i  min_b1 = voxelFilterOriginal.getMinBoxCoordinates ();
//        Eigen::Vector3i  max_b1 = voxelFilterOriginal.getMaxBoxCoordinates ();

    float MatchedVoxels=0 ;//OriginalVoxelsSize=0, ;

        // iterate over the entire original voxel grid to get the size of the grid
//        for (int kk = min_b1.z (); kk <= max_b1.z (); ++kk)
//        {
//            for (int jj = min_b1.y (); jj <= max_b1.y (); ++jj)
//            {
//                for (int ii = min_b1.x (); ii <= max_b1.x (); ++ii)
//                {
//                    Eigen::Vector3i ijk1 (ii, jj, kk);
//                    int index1 = voxelFilterOriginal.getCentroidIndexAt (ijk1);
//                    if(index1!=-1)
//                    {
//                        OriginalVoxelsSize++;
//                    }

//                }
//            }
//        }

        //iterate through the entire coverage grid to check the number of matched voxel between the original and the covered ones
    for (int kk = min_b.z (); kk <= max_b.z (); ++kk)
    {
        for (int jj = min_b.y (); jj <= max_b.y (); ++jj)
        {
            for (int ii = min_b.x (); ii <= max_b.x (); ++ii)
            {

                Eigen::Vector3i ijk (ii, jj, kk);
                int index1 = voxelFilterOccupied.getCentroidIndexAt (ijk);
                if(index1!=-1)
                {
                    Eigen::Vector4f centroid = voxelFilterOccupied.getCentroidCoordinate (ijk);
                    Eigen::Vector3i ijk_in_Original= voxelFilterOriginal.getGridCoordinates(centroid[0],centroid[1],centroid[2]) ;

                    int index = voxelFilterOriginal.getCentroidIndexAt (ijk_in_Original);

                    if(index!=-1)
                    {
                        MatchedVoxels++;
                    }
                }

            }
        }
    }

    //calculating the coverage percentage
    float coverage_ratio= MatchedVoxels/OriginalVoxelsSize;
    float coverage_percentage= coverage_ratio*100;

//    std::cout<<" the coverage ratio is = "<<coverage_ratio<<"\n";
//    std::cout<<" the number of covered voxels = "<<MatchedVoxels<<" voxel is covered"<<"\n";
//    std::cout<<" the number of original voxels = "<<OriginalVoxelsSize<<" voxel"<<"\n\n\n";
//    std::cout<<" the coverage percentage is = "<<coverage_percentage<<" %"<<"\n";

    ros::Time covpercent_end = ros::Time::now();
    double elapsed =  covpercent_end.toSec() - covpercent_begin.toSec();
//    std::cout<<"Coverage Percentage Calculation duration (s) = "<<elapsed<<"\n";

    return coverage_percentage;
}
double OcclusionCulling::calcAvgAccuracy(pcl::PointCloud<pcl::PointXYZ> pointCloud)
{
    double avgAccuracy;
    double pointError,val,errorSum=0, errorRatio;
    for (int j=0; j<pointCloud.size(); j++)
    {
        val = pointCloud.at(j).z;//depth
        pointError= 0.0000285 * val * val;
//        errorRatio=pointError/maxAccuracyError;
        errorSum += pointError;
    }
    avgAccuracy=errorSum/pointCloud.size();
    return avgAccuracy;
}
double OcclusionCulling::calcAvgAccuracy(pcl::PointCloud<pcl::PointXYZ> pointCloud, geometry_msgs::Pose cameraPose)
{

        double avgAccuracy;
        double pointError,val,errorSum=0;
        pcl::PointCloud<pcl::PointXYZ> transformedPointCloud;
        transformedPointCloud = pointCloudViewportTransform(pointCloud, cameraPose);

        for (int j=0; j<transformedPointCloud.size(); j++)
        {
            val = transformedPointCloud.at(j).x;//depth (it is at x axis because of the frustum culling camera pose requirement)
            pointError= 0.0000285 * val * val;
            errorSum += pointError;
        }
        avgAccuracy=errorSum/transformedPointCloud.size();
        return avgAccuracy;
}

void OcclusionCulling::SSMaxMinAccuracy(std::vector<geometry_msgs::PoseArray> sensorsPoses)
{

    pcl::PointCloud<pcl::PointXYZ> global, globalVis;
    double max=0,min=std::numeric_limits<double>::max();

    for (int i = 0 ; i<sensorsPoses.size(); i++)
    {
        for (int j = 0; j<sensorsPoses[i].poses.size(); j++)
        {
                tf::Quaternion qt(sensorsPoses[i].poses[j].orientation.x, sensorsPoses[i].poses[j].orientation.y, sensorsPoses[i].poses[j].orientation.z, sensorsPoses[i].poses[j].orientation.w) ;
                double r, p, y;
                tf::Matrix3x3(qt).getRPY(r, p, y);
                //std::cout<<" camera: "<<r<<" "<<p<<" "<<y<<std::endl;
                //std::cout<<" camera: "<<r*180/M_PI<<" "<<p*180/M_PI<<" "<<y*180/M_PI<<std::endl;

                pcl::PointCloud<pcl::PointXYZ> visible;
                pcl::PointCloud<pcl::PointXYZ> transformedVisible;

                visible += extractVisibleSurface(sensorsPoses[i].poses[j]);

                transformedVisible = pointCloudViewportTransform(visible, sensorsPoses[i].poses[j]);

                global +=transformedVisible;
                globalVis += visible;

                for(int i=0; i<transformedVisible.points.size();i++){
                    double temp = transformedVisible.points[i].x;//depth (it is at x axis because of the frustum culling camera pose requirement)
                    //std::cout<<"depth are :  "<<temp<<"points\n";
                    if(max<temp)
                        max=temp;
                    if(min>temp)
                        min=temp;
                }
        }

    }


    maxAccuracyError = 0.0000285 * max*max;// the standard deviation equation is taken from paper
    minAccuracyError = 0.0000285 * min*min;
    std::cout<<"Maximum error: "<<maxAccuracyError<<" for the depth of: "<<max<<"\n";
    std::cout<<"Minimum error: "<<minAccuracyError<<" for the depth of: "<<min<<"\n";

    AccuracyMaxSet = true;

}

void OcclusionCulling::transformPointMatVec(tf::Vector3 translation, tf::Matrix3x3 rotation, geometry_msgs::Point32 in, geometry_msgs::Point32& out)
{

    double x = rotation[0].x() * in.x + rotation[0].y() * in.y + rotation[0].z() * in.z + translation.x();
    double y = rotation[1].x() * in.x + rotation[1].y() * in.y + rotation[1].z() * in.z + translation.y();
    double z = rotation[2].x() * in.x + rotation[2].y() * in.y + rotation[2].z() * in.z + translation.z();

    out.x = x;
    out.y = y;
    out.z = z;
}


//translate the pcd viewport (0,0,0) to the camera viewport (viewpoints)
//All pcd files have viewports set to (0,0,0) ... occlusion culling extract the point cloud but doesn't change the point cloud depth
//This function will transform the viewport to the new viewport
pcl::PointCloud<pcl::PointXYZ> OcclusionCulling::pointCloudViewportTransform(pcl::PointCloud<pcl::PointXYZ> pointCloud, geometry_msgs::Pose cameraPose)
{

    pcl::PointCloud<pcl::PointXYZ> posArray;
    tf::Matrix3x3 rotZ, rotX, rotY, rotE;

    //traslation accroding to the camera position
    tf::Vector3 cameraPoseTrans(-1*cameraPose.position.x, -1* cameraPose.position.y, -1*cameraPose.position.z);
    //std::cout<<"camera position: "<< cameraPose.position.x<<" "<<cameraPose.position.y<<" "<< cameraPose.position.z<<std::endl;

    //vector and matrice to help in performing no translation or no rotation
    tf::Vector3 transE(0, 0, 0); //No translation
    //No Rotation
    rotE.setValue(1,0,0,
                  0,1,0,
                  0,0,1);

    // rotation of the uav interms of the previous viewport of the pointcloud
    tf::Quaternion qt(cameraPose.orientation.x, cameraPose.orientation.y, cameraPose.orientation.z, cameraPose.orientation.w) ;
    double r, p, y;
    tf::Matrix3x3(qt).getRPY(r, p, y);
    double yaw = -1 * y;
    //std::cout<<" yaw angle: "<<yaw<<std::endl;
    //std::cout<<" cos angle: "<<std::cos(yaw)<<std::endl;
    //std::cout<<" sin angle: "<<std::sin(yaw)<<std::endl;

    rotZ.setValue(std::cos(yaw),-1*std::sin(yaw),0,
                  std::sin(yaw),std::cos(yaw),0,
                  0, 0, 1);

    // rotation required for frustum culling (not required !)
    double roll = -1 * 90;
    rotX.setValue(1,0,0,
                  0, std::cos(roll),-1*std::sin(roll),
                  0, std::sin(roll),std::cos(roll)  );

    // rotation for the camera orientation
    double pitch = -1 * p;
    //std::cout<<" pitch angle: "<<pitch<<std::endl;

    rotY.setValue(std::cos(pitch),0,std::sin(pitch),
                  0,1,0,
                  -1*std::sin(pitch), 0, std::cos(pitch));

    for (int i = 0; i < pointCloud.size() ; i++) {

        geometry_msgs::Point32 ptIN, ptOUT, ptOUT1, ptOUT2, ptOUT3 ;
        ptIN.x = pointCloud.points[i].data[0];
        ptIN.y = pointCloud.points[i].data[1];
        ptIN.z = pointCloud.points[i].data[2];

        //std::cout<<"camera position: "<< cameraPose.position.x<<" "<<cameraPose.position.y<<" "<< cameraPose.position.z<<std::endl;
        //std::cout<<"point in : "<<ptIN.x<<" "<<ptIN.y << " "<<ptIN.z<<" "<<std::endl;

        //translation to camera position
        transformPointMatVec(cameraPoseTrans, rotE, ptIN, ptOUT);
        //std::cout<<"point out1 : "<<ptOUT.x<<" "<<ptOUT.y << " "<<ptOUT.z<<" "<<std::endl;

        //rotation around z (yaw) according to the camera orientation
        transformPointMatVec(transE, rotZ, ptOUT, ptOUT1);
        //std::cout<<"point out2 : "<<ptOUT1.x<<" "<<ptOUT1.y << " "<<ptOUT1.z<<" "<<std::endl;

        //rotation around y (pitch) according to the camera tilt
        transformPointMatVec(transE, rotY, ptOUT1, ptOUT2);
        //std::cout<<"point out3 : "<<ptOUT2.x<<" "<<ptOUT2.y << " "<<ptOUT2.z<<" "<<std::endl;

        //rotation around x (roll)
        //transformPointMatVec(transE, rotX, ptOUT2, ptOUT3);
        //std::cout<<"point out4 : "<<ptOUT3.x<<" "<<ptOUT3.y << " "<<ptOUT3.z<<" "<<std::endl;

        pcl::PointXYZ finalPt;
        finalPt.data[0] = ptOUT2.x;
        finalPt.data[1] = ptOUT2.y;
        finalPt.data[2] = ptOUT2.z;
        posArray.points.push_back(finalPt);

    }

    posArray.header = pointCloud.header;
    return posArray;
}



void OcclusionCulling::visualizeFOV(geometry_msgs::Pose location)
{

    pcl::PointCloud <pcl::PointXYZ>::Ptr output (new pcl::PointCloud <pcl::PointXYZ>);

    Eigen::Matrix4f camera_pose;
    Eigen::Matrix3d Rd;
    Eigen::Matrix3f Rf;

    camera_pose.setZero ();

    tf::Quaternion qt;
    qt.setX(location.orientation.x);
    qt.setY(location.orientation.y);
    qt.setZ(location.orientation.z);
    qt.setW(location.orientation.w);
    tf::Matrix3x3 R_tf(qt);
    tf::matrixTFToEigen(R_tf,Rd);
    Rf = Rd.cast<float>();
    camera_pose.block (0, 0, 3, 3) = Rf;
    Eigen::Vector3f T;
    T (0) = location.position.x; T (1) = location.position.y; T (2) = location.position.z;
    camera_pose.block (0, 3, 3, 1) = T;
    camera_pose (3, 3) = 1;
    fc.setCameraPose (camera_pose);
    fc.filter (*output);

    //*** visualization the FOV *****
    std::vector<geometry_msgs::Point> fov_points;
    int c_color[3];
    geometry_msgs::Point point1;
    point1.x=fc.fp_bl[0];point1.y=fc.fp_bl[1];point1.z=fc.fp_bl[2]; fov_points.push_back(point1);//0
    point1.x=fc.fp_br[0];point1.y=fc.fp_br[1];point1.z=fc.fp_br[2]; fov_points.push_back(point1);//1
    point1.x=fc.fp_tr[0];point1.y=fc.fp_tr[1];point1.z=fc.fp_tr[2]; fov_points.push_back(point1);//2
    point1.x=fc.fp_tl[0];point1.y=fc.fp_tl[1];point1.z=fc.fp_tl[2]; fov_points.push_back(point1);//3
    point1.x=fc.np_bl[0];point1.y=fc.np_bl[1];point1.z=fc.np_bl[2]; fov_points.push_back(point1);//4
    point1.x=fc.np_br[0];point1.y=fc.np_br[1];point1.z=fc.np_br[2]; fov_points.push_back(point1);//5
    point1.x=fc.np_tr[0];point1.y=fc.np_tr[1];point1.z=fc.np_tr[2]; fov_points.push_back(point1);//6
    point1.x=fc.np_tl[0];point1.y=fc.np_tl[1];point1.z=fc.np_tl[2]; fov_points.push_back(point1);//7

    std::vector<geometry_msgs::Point> fov_linesNear;
    fov_linesNear.push_back(fov_points[4]);fov_linesNear.push_back(fov_points[5]);
    fov_linesNear.push_back(fov_points[5]);fov_linesNear.push_back(fov_points[6]);
    fov_linesNear.push_back(fov_points[6]);fov_linesNear.push_back(fov_points[7]);
    fov_linesNear.push_back(fov_points[7]);fov_linesNear.push_back(fov_points[4]);
    c_color[0]=1; c_color[1]=0; c_color[2]=1;
    linesList1 = drawLines(fov_linesNear,id++,c_color);//purple

    std::vector<geometry_msgs::Point> fov_linesFar;
    fov_linesFar.push_back(fov_points[0]);fov_linesFar.push_back(fov_points[1]);
    fov_linesFar.push_back(fov_points[1]);fov_linesFar.push_back(fov_points[2]);
    fov_linesFar.push_back(fov_points[2]);fov_linesFar.push_back(fov_points[3]);
    fov_linesFar.push_back(fov_points[3]);fov_linesFar.push_back(fov_points[0]);
    c_color[0]=1; c_color[1]=1; c_color[2]=0;
    linesList2 = drawLines(fov_linesFar,id++,c_color);//yellow


    std::vector<geometry_msgs::Point> fov_linestop;
    fov_linestop.push_back(fov_points[7]);fov_linestop.push_back(fov_points[3]);//top
    fov_linestop.push_back(fov_points[6]);fov_linestop.push_back(fov_points[2]);//top
    c_color[0]=0; c_color[1]=1; c_color[2]=0;
    linesList3 = drawLines(fov_linestop,id++,c_color);//green

    std::vector<geometry_msgs::Point> fov_linesbottom;
    fov_linesbottom.push_back(fov_points[5]);fov_linesbottom.push_back(fov_points[1]);//bottom
    fov_linesbottom.push_back(fov_points[4]);fov_linesbottom.push_back(fov_points[0]);//bottom
    c_color[0]=0; c_color[1]=0; c_color[2]=1;
    linesList4 = drawLines(fov_linesbottom,id++,c_color);//blue

    marker_array.markers.push_back(linesList1);
    marker_array.markers.push_back(linesList2);
    marker_array.markers.push_back(linesList3);
    marker_array.markers.push_back(linesList4);
    fov_pub.publish(marker_array);
}
visualization_msgs::Marker OcclusionCulling::drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color[])
{
    visualization_msgs::Marker linksMarkerMsg;
    linksMarkerMsg.header.frame_id=frame_id;
    linksMarkerMsg.header.stamp=ros::Time::now();
    linksMarkerMsg.ns="link_marker";
    linksMarkerMsg.id = id;
    linksMarkerMsg.type = visualization_msgs::Marker::LINE_LIST;
    linksMarkerMsg.scale.x = 0.08;//0.03
    linksMarkerMsg.action  = visualization_msgs::Marker::ADD;
    linksMarkerMsg.lifetime  = ros::Duration(1000);
    std_msgs::ColorRGBA color;
    color.r = (float)c_color[0]; color.g=(float)c_color[1]; color.b=(float)c_color[2], color.a=1.0f;

    std::vector<geometry_msgs::Point>::iterator linksIterator;
    for(linksIterator = links.begin();linksIterator != links.end();linksIterator++)
    {
        linksMarkerMsg.points.push_back(*linksIterator);
        linksMarkerMsg.colors.push_back(color);
    }
   return linksMarkerMsg;
}
