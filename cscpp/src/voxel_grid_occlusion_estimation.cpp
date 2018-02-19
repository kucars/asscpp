/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author : Christian Potthast
 * Email  : potthast@usc.edu
 *
 */

#include <pcl/common/common.h>
#include <cscpp/voxel_grid_occlusion_estimation.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::VoxelGridOcclusionEstimationT::initializeVoxelGrid ()
{
  // initialization set to true
  initialized_ = true;
  
  // create the voxel grid and store the output cloud
  this->filter (filtered_cloud_);

  // Get the minimum and maximum bounding box dimensions
  b_min_[0] = (static_cast<float> ( min_b_[0]) * leaf_size_[0]);
  b_min_[1] = (static_cast<float> ( min_b_[1]) * leaf_size_[1]);
  b_min_[2] = (static_cast<float> ( min_b_[2]) * leaf_size_[2]);
  b_max_[0] = (static_cast<float> ( (max_b_[0]) + 1) * leaf_size_[0]);
  b_max_[1] = (static_cast<float> ( (max_b_[1]) + 1) * leaf_size_[1]);
  b_max_[2] = (static_cast<float> ( (max_b_[2]) + 1) * leaf_size_[2]);

//  std::cout<<"b_min X:"<<b_min_[0]<<" y:"<< b_min_[1]<<" z:"<< b_min_[2]<<"\n";
//  std::cout<<"b_max X:"<<b_max_[0]<<" y:"<< b_max_[1]<<" z:"<< b_max_[2]<<"\n";
  // set the sensor origin and sensor orientation
  sensor_origin_ = filtered_cloud_.sensor_origin_;
  sensor_orientation_ = filtered_cloud_.sensor_orientation_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::VoxelGridOcclusionEstimationT::occlusionEstimation (int& out_state,
                                                                const Eigen::Vector3i& in_target_voxel)
{
  if (!initialized_)
  {
    PCL_ERROR ("Voxel grid not initialized; call initializeVoxelGrid () first! \n");
    return -1;
  }

  // estimate direction to target voxel
  Eigen::Vector4f p = getCentroidCoordinate (in_target_voxel);
  Eigen::Vector4f direction = p - sensor_origin_;
  direction.normalize ();

  // estimate entry point into the voxel grid
  float tmin = rayBoxIntersection (sensor_origin_, direction);

  if (tmin == -1)
  {
    PCL_ERROR ("The ray does not intersect with the bounding box \n");
    return -1;
  }

  // ray traversal
  out_state = rayTraversal (in_target_voxel, sensor_origin_, direction, tmin);

  return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::VoxelGridOcclusionEstimationT::occlusionEstimation (int& out_state,
                                                                std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> >& out_ray,
                                                                const Eigen::Vector3i& in_target_voxel)
{
  if (!initialized_)
  {
    PCL_ERROR ("Voxel grid not initialized; call initializeVoxelGrid () first! \n");
    return -1;
  }

  // estimate direction to target voxel
  Eigen::Vector4f p = getCentroidCoordinate (in_target_voxel);
  Eigen::Vector4f direction = p - sensor_origin_;
  direction.normalize ();

  // estimate entry point into the voxel grid
  float tmin = rayBoxIntersection (sensor_origin_, direction);

  if (tmin == -1)
  {
    PCL_ERROR ("The ray does not intersect with the bounding box \n");
    return -1;
  }

  // ray traversal
  out_state = rayTraversal (out_ray, in_target_voxel, sensor_origin_, direction, tmin);

  return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::VoxelGridOcclusionEstimationT::occlusionEstimationAll (std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> >& occluded_voxels)
{
  if (!initialized_)
  {
    PCL_ERROR ("Voxel grid not initialized; call initializeVoxelGrid () first! \n");
    return -1;
  }

  // reserve space for the ray vector
  int reserve_size = div_b_[0] * div_b_[1] * div_b_[2];
  occluded_voxels.reserve (reserve_size);

  // iterate over the entire voxel grid
  for (int kk = min_b_.z (); kk <= max_b_.z (); ++kk)
    for (int jj = min_b_.y (); jj <= max_b_.y (); ++jj)
      for (int ii = min_b_.x (); ii <= max_b_.x (); ++ii)
      {
        Eigen::Vector3i ijk (ii, jj, kk);
        // process all free voxels
        int index = this->getCentroidIndexAt (ijk);
        if (index == -1)
        {
          // estimate direction to target voxel
          Eigen::Vector4f p = getCentroidCoordinate (ijk);
          Eigen::Vector4f direction = p - sensor_origin_;
          direction.normalize ();
          
          // estimate entry point into the voxel grid
          float tmin = rayBoxIntersection (sensor_origin_, direction);

          // ray traversal
          int state = rayTraversal (ijk, sensor_origin_, direction, tmin);
          
          // if voxel is occluded
          if (state == 1)
            occluded_voxels.push_back (ijk);
        }
      }
  return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float
pcl::VoxelGridOcclusionEstimationT::rayBoxIntersection (const Eigen::Vector4f& origin, 
                                                               const Eigen::Vector4f& direction)
{
  float tmin, tmax, tymin, tymax, tzmin, tzmax;

  if (direction[0] >= 0)
  {
    tmin = (b_min_[0] - origin[0]) / direction[0];
    tmax = (b_max_[0] - origin[0]) / direction[0];
  }
  else
  {
    tmin = (b_max_[0] - origin[0]) / direction[0];
    tmax = (b_min_[0] - origin[0]) / direction[0];
  }

  if (direction[1] >= 0)
  {
    tymin = (b_min_[1] - origin[1]) / direction[1];
    tymax = (b_max_[1] - origin[1]) / direction[1]; 
  }
  else
  {
    tymin = (b_max_[1] - origin[1]) / direction[1];
    tymax = (b_min_[1] - origin[1]) / direction[1];
  }

  if ((tmin > tymax) || (tymin > tmax))
  {
    PCL_ERROR ("no intersection with the bounding box \n");
    tmin = -1.0f;
    return tmin;
  }

  if (tymin > tmin)
    tmin = tymin;
  if (tymax < tmax)
    tmax = tymax;

  if (direction[2] >= 0)
  {
    tzmin = (b_min_[2] - origin[2]) / direction[2];
    tzmax = (b_max_[2] - origin[2]) / direction[2];
  }
  else
  {
    tzmin = (b_max_[2] - origin[2]) / direction[2];
    tzmax = (b_min_[2] - origin[2]) / direction[2];
  }

  if ((tmin > tzmax) || (tzmin > tmax))
  {
    PCL_ERROR ("no intersection with the bounding box \n");
    tmin = -1.0f;
    return tmin;       
  }

  if (tzmin > tmin)
    tmin = tzmin;
  if (tzmax < tmax)
    tmax = tzmax;
//  std::cout<<"Tmin is:"<<tmin<<"\n";
  return tmin;
}

float
pcl::VoxelGridOcclusionEstimationT::rayBoxIntersection (const Eigen::Vector4f& origin,
                                                               const Eigen::Vector4f& direction, pcl::PointXYZ &minPoint,pcl::PointXYZ &maxPoint)
{
  float tmin, tmax, tymin, tymax, tzmin, tzmax;

  if (direction[0] >= 0)
  {
    tmin = (b_min_[0] - origin[0]) / direction[0];
    tmax = (b_max_[0] - origin[0]) / direction[0];
  }
  else
  {
    tmin = (b_max_[0] - origin[0]) / direction[0];
    tmax = (b_min_[0] - origin[0]) / direction[0];
  }

  minPoint.x =  tmin;
  maxPoint.x =  tmax;

  if (direction[1] >= 0)
  {
    tymin = (b_min_[1] - origin[1]) / direction[1];
    tymax = (b_max_[1] - origin[1]) / direction[1];
  }
  else
  {
    tymin = (b_max_[1] - origin[1]) / direction[1];
    tymax = (b_min_[1] - origin[1]) / direction[1];
  }

  minPoint.y =  tymin;
  maxPoint.y =  tymax;
  
  if ((tmin > tymax) || (tymin > tmax))
  {
    PCL_ERROR ("no intersection with the bounding box \n");
    tmin = -1.0f;
    return tmin;
  }

  if (tymin > tmin)
    tmin = tymin;
  if (tymax < tmax)
    tmax = tymax;

  if (direction[2] >= 0)
  {
    tzmin = (b_min_[2] - origin[2]) / direction[2];
    tzmax = (b_max_[2] - origin[2]) / direction[2];
  }
  else
  {
    tzmin = (b_max_[2] - origin[2]) / direction[2];
    tzmax = (b_min_[2] - origin[2]) / direction[2];
  }

  minPoint.z =  tzmin;
  maxPoint.z =  tzmax;
  
  if ((tmin > tzmax) || (tzmin > tmax))
  {
    PCL_ERROR ("no intersection with the bounding box \n");
    tmin = -1.0f;
    return tmin;
  }

  if (tzmin > tmin)
    tmin = tzmin;
  if (tzmax < tmax)
    tmax = tzmax;
  //std::cout<<"Tmin is:"<<tmin<<"\n";
  return tmin;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::VoxelGridOcclusionEstimationT::rayTraversal (const Eigen::Vector3i& target_voxel,
                                                         const Eigen::Vector4f& origin, 
                                                         const Eigen::Vector4f& direction,
                                                         const float t_min)
{
  // coordinate of the boundary of the voxel grid
  Eigen::Vector4f start = origin + t_min * direction;

  // i,j,k coordinate of the voxel were the ray enters the voxel grid
  Eigen::Vector3i ijk = getGridCoordinatesRound (start[0], start[1], start[2]);

  // steps in which direction we have to travel in the voxel grid
  int step_x, step_y, step_z;

  // centroid coordinate of the entry voxel
  Eigen::Vector4f voxel_max = getCentroidCoordinate (ijk);

  if (direction[0] >= 0)
  {
    voxel_max[0] += leaf_size_[0] * 0.5f;
    step_x = 1;
  }
  else
  {
    voxel_max[0] -= leaf_size_[0] * 0.5f;
    step_x = -1;
  }
  if (direction[1] >= 0)
  {
    voxel_max[1] += leaf_size_[1] * 0.5f;
    step_y = 1;
  }
  else
  {
    voxel_max[1] -= leaf_size_[1] * 0.5f;
    step_y = -1;
  }
  if (direction[2] >= 0)
  {
    voxel_max[2] += leaf_size_[2] * 0.5f;
    step_z = 1;
  }
  else
  {
    voxel_max[2] -= leaf_size_[2] * 0.5f;
    step_z = -1;
  }

  float t_max_x = t_min + (voxel_max[0] - start[0]) / direction[0];
  float t_max_y = t_min + (voxel_max[1] - start[1]) / direction[1];
  float t_max_z = t_min + (voxel_max[2] - start[2]) / direction[2];
     
  float t_delta_x = leaf_size_[0] / static_cast<float> (fabs (direction[0]));
  float t_delta_y = leaf_size_[1] / static_cast<float> (fabs (direction[1]));
  float t_delta_z = leaf_size_[2] / static_cast<float> (fabs (direction[2]));

  // index of the point in the point cloud
  int index;

  while ( (ijk[0] < max_b_[0]+1) && (ijk[0] >= min_b_[0]) && 
          (ijk[1] < max_b_[1]+1) && (ijk[1] >= min_b_[1]) && 
          (ijk[2] < max_b_[2]+1) && (ijk[2] >= min_b_[2]) )
  {
    // check if we reached target voxel
    if (ijk[0] == target_voxel[0] && ijk[1] == target_voxel[1] && ijk[2] == target_voxel[2])
      return 0;

    // check if voxel is occupied, if yes return 1 for occluded
    index = this->getCentroidIndexAt (ijk);
    if (index != -1)
      return 1;

    // estimate next voxel
    if(t_max_x <= t_max_y && t_max_x <= t_max_z)
    {
      t_max_x += t_delta_x;
      ijk[0] += step_x;
    }
    else if(t_max_y <= t_max_z && t_max_y <= t_max_x)
    {
      t_max_y += t_delta_y;
      ijk[1] += step_y;
    }
    else
    {
      t_max_z += t_delta_z;
      ijk[2] += step_z;
    }
  }
  return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::VoxelGridOcclusionEstimationT::rayTraversal (std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> >& out_ray,
                                                         const Eigen::Vector3i& target_voxel,
                                                         const Eigen::Vector4f& origin, 
                                                         const Eigen::Vector4f& direction,
                                                         const float t_min)
{
  // reserve space for the ray vector
  int reserve_size = div_b_.maxCoeff () * div_b_.maxCoeff ();
  out_ray.reserve (reserve_size);
//  std::cout<<"t_min is x:"<<t_min<<"\n";
  // coordinate of the boundary of the voxel grid
  Eigen::Vector4f start = origin + t_min * direction;
  
  // i,j,k coordinate of the voxel were the ray enters the voxel grid
  Eigen::Vector3i ijk = getGridCoordinatesRound (start[0], start[1], start[2]);
  //Eigen::Vector3i ijk = this->getGridCoordinates (start_x, start_y, start_z);

  // steps in which direction we have to travel in the voxel grid
  int step_x, step_y, step_z;
  int step_x2, step_y2, step_z2;

  // centroid coordinate of the entry voxel
  Eigen::Vector4f voxel_max = getCentroidCoordinate (ijk);
//  std::cout<<"Direction is x:"<<direction[0]<<" y:"<<direction[1]<<" z:"<<direction[2]<<"\n";
  
  if (direction[0] >= 0)
  {
    voxel_max[0] += leaf_size_[0] * 0.5f;
    step_x = 1;
  }
  else
  {
    voxel_max[0] -= leaf_size_[0] * 0.5f;
    step_x = -1;
  }
  if (direction[1] >= 0)
  {
    voxel_max[1] += leaf_size_[1] * 0.5f;
    step_y = 1;
  }
  else
  {
    voxel_max[1] -= leaf_size_[1] * 0.5f;
    step_y = -1;
  }
  if (direction[2] >= 0)
  {
    voxel_max[2] += leaf_size_[2] * 0.5f;
    step_z = 1;
  }
  else
  {
    voxel_max[2] -= leaf_size_[2] * 0.5f;
    step_z = -1;
  }
  /*
  Eigen::Vector4f target = getCentroidCoordinate (target_voxel);
  double dist = ((start.array () - target.array ()).abs ().sum ());
  int numOfSteps = ceil(dist/leaf_size_[0]);
  std::cout<<"Length of Ray is:"<<dist<<" number of Steps:"<<numOfSteps<<"\n";

  out_ray.push_back(ijk);

  Eigen::Vector4f dir2 = target - start;
  if (dir2[0] >= 0)
  {
    step_x2 = -1;
  }
  else
  {
    step_x2 = 1;
  }
  if (dir2[1] >= 0)
  {
    step_y2 = -1;
  }
  else
  {
    step_y2 = 1;
  }
  if (dir2[2] >= 0)
  {
    step_z2 = -1;
  }
  else
  {
    step_z2 = 1;
  }
  for(int i=1;i<=numOfSteps;i++)
  {
    Eigen::Vector4f nextPose;
    nextPose[0] = start[0] + i*step_x2*leaf_size_[0]*dir2[0];
    nextPose[1] = start[1] + i*step_y2*leaf_size_[1]*dir2[1];
    nextPose[2] = start[2] + i*step_z2*leaf_size_[2]*dir2[2];
    Eigen::Vector3i voxelIJK = getGridCoordinatesRound (nextPose[0], nextPose[1], nextPose[2]);
    out_ray.push_back (voxelIJK);
    std::cout<<"IN - Next Pose X:"<<nextPose[0]<<" y:"<< nextPose[1]<<" z:"<< nextPose[2]<<"\n";
    if (voxelIJK[0] == target_voxel[0] && voxelIJK[1] == target_voxel[1] && voxelIJK[2] == target_voxel[2])
      break;
  }

*/
  float t_max_x = t_min + (voxel_max[0] - start[0]) / direction[0];
  float t_max_y = t_min + (voxel_max[1] - start[1]) / direction[1];
  float t_max_z = t_min + (voxel_max[2] - start[2]) / direction[2];
     
  float t_delta_x = leaf_size_[0] / static_cast<float> (fabs (direction[0]));
  float t_delta_y = leaf_size_[1] / static_cast<float> (fabs (direction[1]));
  float t_delta_z = leaf_size_[2] / static_cast<float> (fabs (direction[2]));

  //if(ijk[0] < max_b_[0]+1)
  /*
  if(ijk[0]!=43)
  {
      std::cout<<"IN - Direction X:"<<direction[0]<<" y:"<< direction[1]<<" z:"<< direction[2]<<"\n";
      std::cout<<"IN - LeafSize X:"<<leaf_size_[0]<<" y:"<< leaf_size_[1]<<" z:"<< leaf_size_[2]<<"\n";
      std::cout<<"IN - Delta X:"<<t_delta_x<<" y:"<< t_delta_y<<" z:"<< t_delta_z<<"\n";

      std::cout<<"IN - IJK X:"<<ijk[0]<<" y:"<< ijk[1]<<" z:"<< ijk[2]<<"\n";
      std::cout<<"IN - MAXb X:"<<max_b_[0]<<" y:"<< max_b_[1]<<" z:"<< max_b_[2]<<"\n";
      std::cout<<"IN - MINb X:"<<min_b_[0]<<" y:"<< min_b_[1]<<" z:"<< min_b_[2]<<"\n";
  }
  */
  // the index of the cloud (-1 if empty)
  int index = -1;
  int result = 0;
  bool hereOnce=false;
  while ( (ijk[0]<= max_b_[0]+1) && (ijk[0]+1 >= min_b_[0]) &&
          (ijk[1]<= max_b_[1]+1) && (ijk[1]+1 >= min_b_[1]) &&
          (ijk[2]<= max_b_[2]+1) && (ijk[2]+1 >= min_b_[2]) )
  {
//      if(!hereOnce)
//      {
//          //std::cout<<"I am here\n";
//          //hereOnce = true;
//      }
    // add voxel to ray
    out_ray.push_back (ijk);

    // check if we reached target voxel
    if (ijk[0] == target_voxel[0] && ijk[1] == target_voxel[1] && ijk[2] == target_voxel[2])
      break;

    // check if voxel is occupied
    index = this->getCentroidIndexAt (ijk);
    if (index != -1)
    {
      Eigen::Vector4f here = getCentroidCoordinate (ijk);
      Eigen::Vector4f target = getCentroidCoordinate (target_voxel);
      double dist = sqrt((here[0] -target[0])*(here[0] -target[0]) + (here[1] -target[1])*(here[1] -target[1]) +(here[2] -target[2])*(here[2] -target[2]));
      if(dist>leaf_size_[0]*2)
        result = 1;
    }
    // estimate next voxel
    if(t_max_x <= t_max_y && t_max_x <= t_max_z)
    {
      t_max_x += t_delta_x;
      ijk[0] += step_x;
    }
    else if(t_max_y <= t_max_z && t_max_y <= t_max_x)
    {
      t_max_y += t_delta_y;
      ijk[1] += step_y;
    }
    else
    {
      t_max_z += t_delta_z;
      ijk[2] += step_z;
    }
  }
  return result;
}
