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
#include <cscpp/voxel_grid_occlusion_estimation_gpu.h>
#include <stdio.h>
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::VoxelGridOcclusionEstimationGPU::initializeVoxelGrid ()
{
  // initialization set to true
  initialized_ = true;

  // create the voxel grid and store the output cloud
  this->filter(filtered_cloud_);

  // Get the minimum and maximum bounding box dimensions
  b_min_[0] = (static_cast<float> ( min_b_[0]) * leaf_size_[0]);
  b_min_[1] = (static_cast<float> ( min_b_[1]) * leaf_size_[1]);
  b_min_[2] = (static_cast<float> ( min_b_[2]) * leaf_size_[2]);
  b_max_[0] = (static_cast<float> ( (max_b_[0]) + 1) * leaf_size_[0]);
  b_max_[1] = (static_cast<float> ( (max_b_[1]) + 1) * leaf_size_[1]);
  b_max_[2] = (static_cast<float> ( (max_b_[2]) + 1) * leaf_size_[2]);

  // calculate the max and min error
  double max=0,min=std::numeric_limits<double>::max();
  for(int i=0; i<filtered_cloud_.points.size();i++){
      double temp = filtered_cloud_.at(i).z;//depth
      if(max<temp)
         max=temp;
      if(min>temp)
         min=temp;
  }
  double maxAccuracyError = 0.0000285 * max*max;
  maxAccuracy(0) = maxAccuracyError;
  double minAccuracyError = 0.0000285 * min*min;
  minAccuracy(0) = minAccuracyError;

  // set the sensor origin and sensor orientation
  sensor_origin_ = filtered_cloud_.sensor_origin_;
  sensor_orientation_ = filtered_cloud_.sensor_orientation_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::VoxelGridOcclusionEstimationGPU::occlusionEstimation (int& out_state,
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
pcl::VoxelGridOcclusionEstimationGPU::occlusionEstimation (int& out_state,
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
pcl::VoxelGridOcclusionEstimationGPU::occlusionEstimationAll (std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> >& occluded_voxels)
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
//                             (deviceX,    deviceY,  deviceZ,        inverse_leaf_size,         leaf_size,         b_min,       min_b,       max_b,         b_max,         sensor_origin,                                                                divb_mul,      occlusionFreePointsIndices,      occlusionFreePointsCount,    numPoints);
__global__ void rayTraversalGPU(double *x, double*y, double*z,float * inverse_leaf_size_,float * leaf_size_,float * b_min_,int * min_b_,int * max_b_,float * b_max_,float * sensor_origin_,float * maxAcc,int * leaf_layout_, int leaf_layout_size, int * divb_mul_,int *occlusionFreePointsIndices, float* occlusionFreePointsEntropies, int* occlusionFreePointsCount,int numPoints)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    //float vLiklihood = 1;
    double DepthAcc, normDepthAcc ;
    float Io, voxelEntrobyIv;
    if(idx<numPoints)
    {
        if(idx==0)
            occlusionFreePointsCount[0] = 0;
        occlusionFreePointsIndices[idx] = -1;
        occlusionFreePointsEntropies[idx] = -1;
        // get ijk
        int i,j,k,a,b,c;
        double cX,cY,cZ,xR,yR,zR,dist;
        i = static_cast<int> (floor (x[idx] * inverse_leaf_size_[0]));
        j = static_cast<int> (floor (y[idx] * inverse_leaf_size_[1]));
        k = static_cast<int> (floor (z[idx] * inverse_leaf_size_[2]));

        // get centroid
        a = ((b_min_[0] < 0) ? (abs (min_b_[0]) + i) : (i - min_b_[0]));
        b = ((b_min_[1] < 0) ? (abs (min_b_[1]) + j) : (j - min_b_[1]));
        c = ((b_min_[2] < 0) ? (abs (min_b_[2]) + k) : (k - min_b_[2]));

        cX = b_min_[0] + (leaf_size_[0] * 0.5f) + (static_cast<float> (a) * leaf_size_[0]);
        cY = b_min_[1] + (leaf_size_[1] * 0.5f) + (static_cast<float> (b) * leaf_size_[1]);
        cZ = b_min_[2] + (leaf_size_[2] * 0.5f) + (static_cast<float> (c) * leaf_size_[2]);

        double directionX = cX - sensor_origin_[0];
        double directionY = cY - sensor_origin_[1];
        double directionZ = cZ - sensor_origin_[2];

        // Temporarily: will consider the cz is the distance from the camera to the voxel centroid (later move to the occlusion culling gpu)
        DepthAcc = 0.0000285 * directionZ * directionZ;

        double directionLen = sqrt(directionX*directionX + directionY*directionY + directionZ*directionZ);
        //Normalize direction
        double direction[3];
        direction[0] = directionX/directionLen;
        direction[1] = directionY/directionLen;
        direction[2] = directionZ/directionLen;

        // Calculate t_min (box intersection)
        float t_min, tmax, tymin, tymax, tzmin, tzmax;
        if (direction[0] >= 0)
        {
            t_min = (b_min_[0] - sensor_origin_[0]) / direction[0];
            tmax  = (b_max_[0] - sensor_origin_[0]) / direction[0];
        }
        else
        {
            t_min = (b_max_[0] - sensor_origin_[0]) / direction[0];
            tmax  = (b_min_[0] - sensor_origin_[0]) / direction[0];
        }

        if (direction[1] >= 0)
        {
            tymin = (b_min_[1] - sensor_origin_[1]) / direction[1];
            tymax = (b_max_[1] - sensor_origin_[1]) / direction[1];
        }
        else
        {
            tymin = (b_max_[1] - sensor_origin_[1]) / direction[1];
            tymax = (b_min_[1] - sensor_origin_[1]) / direction[1];
        }

        if ((t_min > tymax) || (tymin > tmax))
        {
            return;
        }

        if (tymin > t_min)
            t_min = tymin;
        if (tymax < tmax)
            tmax = tymax;

        if (direction[2] >= 0)
        {
            tzmin = (b_min_[2] - sensor_origin_[2]) / direction[2];
            tzmax = (b_max_[2] - sensor_origin_[2]) / direction[2];
        }
        else
        {
            tzmin = (b_max_[2] - sensor_origin_[2]) / direction[2];
            tzmax = (b_min_[2] - sensor_origin_[2]) / direction[2];
        }

        if ((t_min > tzmax) || (tzmin > tmax))
        {
            return;
        }

        if (tzmin > t_min)
            t_min = tzmin;
        if (tzmax < tmax)
            tmax = tzmax;

        // RayTraversal

        // coordinate of the boundary of the voxel grid

        double start[3];
        start[0] = sensor_origin_[0] + t_min * direction[0];
        start[1] = sensor_origin_[1] + t_min * direction[1];
        start[2] = sensor_origin_[2] + t_min * direction[2];

        // i,j,k coordinate of the voxel were the ray enters the voxel grid

        double iR = static_cast<int> (round (start[0] * inverse_leaf_size_[0]));
        double jR = static_cast<int> (round (start[1] * inverse_leaf_size_[1]));
        double kR = static_cast<int> (round (start[2] * inverse_leaf_size_[2]));

        // steps in which direction we have to travel in the voxel grid
        int step_x, step_y, step_z;

        // centroid coordinate of the entry voxel
        double voxel_max[3];
        a = ((b_min_[0] < 0) ? (abs (min_b_[0]) + iR) : (iR - min_b_[0]));
        b = ((b_min_[1] < 0) ? (abs (min_b_[1]) + jR) : (jR - min_b_[1]));
        c = ((b_min_[2] < 0) ? (abs (min_b_[2]) + kR) : (kR - min_b_[2]));

        voxel_max[0] = b_min_[0] + (leaf_size_[0] * 0.5f) + (static_cast<float> (a) * leaf_size_[0]);
        voxel_max[1] = b_min_[1] + (leaf_size_[1] * 0.5f) + (static_cast<float> (b) * leaf_size_[1]);
        voxel_max[2] = b_min_[2] + (leaf_size_[2] * 0.5f) + (static_cast<float> (c) * leaf_size_[2]);

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

        // the index of the cloud (-1 if empty)
        int index = -1;
        int result = 0;
        while ( (iR<= max_b_[0]+1) && (iR+1 >= min_b_[0]) &&
                (jR<= max_b_[1]+1) && (jR+1 >= min_b_[1]) &&
                (kR<= max_b_[2]+1) && (kR+1 >= min_b_[2]) )
        {
            // check if we reached target voxel
            if (i == iR && j == jR && k == kR)
                break;
            // check if voxel is occupied
            int ii = (iR - min_b_[0])*divb_mul_[0] + (jR - min_b_[1])*divb_mul_[1] + (kR - min_b_[2])*divb_mul_[2];
            if (ii < 0 || ii >= leaf_layout_size)
            {
                index = -1;
                //multiply (1 - unoccupied Prob.) of unoccupied voxels
                //vLiklihood *= unOccupiedPPar_[0];
            }
            else
                 index = leaf_layout_[ii];
            if(index!=-1)
            {
                a = ((b_min_[0] < 0) ? (abs (min_b_[0]) + iR) : (iR - min_b_[0]));
                b = ((b_min_[1] < 0) ? (abs (min_b_[1]) + jR) : (jR - min_b_[1]));
                c = ((b_min_[2] < 0) ? (abs (min_b_[2]) + kR) : (kR - min_b_[2]));

                xR = b_min_[0] + (leaf_size_[0] * 0.5f) + (static_cast<float> (a) * leaf_size_[0]);
                yR = b_min_[1] + (leaf_size_[1] * 0.5f) + (static_cast<float> (b) * leaf_size_[1]);
                zR = b_min_[2] + (leaf_size_[2] * 0.5f) + (static_cast<float> (c) * leaf_size_[2]);

                dist = sqrt((xR  -cX)*(xR - cX) + (yR - cY)*(yR - cY) + (zR -cZ)*(zR -cZ));
                if(dist>leaf_size_[0]*2)
                {
                    result = 1;
                    break;
                }


            }
            // estimate next voxel
            if(t_max_x <= t_max_y && t_max_x <= t_max_z)
            {
                t_max_x += t_delta_x;
                iR += step_x;
            }
            else if(t_max_y <= t_max_z && t_max_y <= t_max_x)
            {
                t_max_y += t_delta_y;
                jR += step_y;
            }
            else
            {
                t_max_z += t_delta_z;
                kR += step_z;
            }
        }
        if(result != 1)
        {
            occlusionFreePointsIndices[idx] = idx;
            //calculate the entroby
            normDepthAcc = (maxAcc[0] - DepthAcc)/maxAcc[0];
            Io = -1*(normDepthAcc)*log(normDepthAcc)-( (1- normDepthAcc)*log((1-normDepthAcc)) );
            //voxelEntrobyIv = Io*vLiklihood;
            voxelEntrobyIv = Io;
            // printf("%f \n", voxelEntrobyIv);
            occlusionFreePointsEntropies[idx] = voxelEntrobyIv;
            atomicAdd(&occlusionFreePointsCount[0], 1);
        }
    }
}

int pcl::VoxelGridOcclusionEstimationGPU::occlusionFreeEstimationAll(pcl::PointCloud <pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>& occlusionFreePoints)
{
    if (!initialized_)
    {
        PCL_ERROR ("Voxel grid not initialized; call initializeVoxelGrid () first! \n");
        return -1;
    }
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > outRay;

    // reserve space for the ray vector
    int reserve_size = div_b_[0] * div_b_[1] * div_b_[2];
    occlusionFreePoints.reserve(reserve_size);
    // iterate over the entire voxel grid

    if(inCloud->points.size() == 0)
        return 0;
    int numPoints = inCloud->points.size();
    int size  = numPoints*sizeof(double);
    int initialPointCount = 0;
    double *deviceX,*deviceY,*deviceZ,*x,*y,*z;
    float *inverse_leaf_size, *leaf_size,*b_min,*sensor_origin,*b_max, *maxAcc, *occlusionFreePointsEntropies;
    int *leaf_layout, *occlusionFreePointsIndices,*occlusionFreePointsCount,*divb_mul,*min_b,*max_b;
    int leaf_layout_size;

    //probabilities used in calculating entroby
    //occupied(0) = 0.5;
    //occupiedPar(0) = 1 - occupied(0);
    //unoccupied(0) = 0.5;
    //unOccupiedPar(0) = 1 - unoccupied(0);

    cudaMalloc((void**)&deviceX, size);
    cudaMalloc((void**)&deviceY, size);
    cudaMalloc((void**)&deviceZ, size);

    x = (double*)malloc(size);
    y = (double*)malloc(size);
    z = (double*)malloc(size);

    cudaMalloc((void**)&inverse_leaf_size, 4*sizeof(float));
    cudaMalloc((void**)&leaf_size,         4*sizeof(float));
    cudaMalloc((void**)&b_min,             4*sizeof(float));
    cudaMalloc((void**)&b_max,             4*sizeof(float));
    cudaMalloc((void**)&sensor_origin,     4*sizeof(float));
    //cudaMalloc((void**)&occupiedP,         4*sizeof(float));
    //cudaMalloc((void**)&occupiedPPar,      4*sizeof(float));
    //cudaMalloc((void**)&unOccupiedPPar,    4*sizeof(float));
    cudaMalloc((void**)&maxAcc,            4*sizeof(float));
    cudaMalloc((void**)&min_b,             4*sizeof(int));
    cudaMalloc((void**)&max_b,             4*sizeof(int));
    cudaMalloc((void**)&divb_mul,          4*sizeof(int));
    cudaMalloc((void**)&leaf_layout, leaf_layout_.size()*sizeof(int));
    cudaMalloc((void**)&occlusionFreePointsIndices, numPoints*sizeof(int));
    cudaMalloc((void**)&occlusionFreePointsEntropies, numPoints*sizeof(float));
    cudaMalloc((void**)&occlusionFreePointsCount, 1*sizeof(int));

    for(int i=0;i<inCloud->points.size();i++)
    {
        x[i] = inCloud->points[i].x;
        y[i] = inCloud->points[i].y;
        z[i] = inCloud->points[i].z;
    }

    cudaMemcpy(deviceX,x,size,cudaMemcpyHostToDevice);
    cudaMemcpy(deviceY,y,size,cudaMemcpyHostToDevice);
    cudaMemcpy(deviceZ,z,size,cudaMemcpyHostToDevice);

    cudaMemcpy(inverse_leaf_size,inverse_leaf_size_.data(),4*sizeof(float),cudaMemcpyHostToDevice);
    cudaMemcpy(leaf_size,leaf_size_.data(),4*sizeof(float),cudaMemcpyHostToDevice);
    cudaMemcpy(b_min,b_min_.data(),4*sizeof(float),cudaMemcpyHostToDevice);
    cudaMemcpy(b_max,b_max_.data(),4*sizeof(float),cudaMemcpyHostToDevice);
    cudaMemcpy(min_b,min_b_.data(),4*sizeof(int),cudaMemcpyHostToDevice);
    cudaMemcpy(max_b,max_b_.data(),4*sizeof(int),cudaMemcpyHostToDevice);
    cudaMemcpy(divb_mul,divb_mul_.data(),4*sizeof(int),cudaMemcpyHostToDevice);
    cudaMemcpy(sensor_origin,sensor_origin_.data(),4*sizeof(float),cudaMemcpyHostToDevice);
    //cudaMemcpy(occupiedP,occupied.data(),4*sizeof(float),cudaMemcpyHostToDevice);
    //cudaMemcpy(occupiedPPar,occupiedPar.data(),4*sizeof(float),cudaMemcpyHostToDevice);
    //cudaMemcpy(unOccupiedPPar,unOccupiedPar.data(),4*sizeof(float),cudaMemcpyHostToDevice);
    cudaMemcpy(maxAcc,maxAccuracy.data(),4*sizeof(float),cudaMemcpyHostToDevice);
    cudaMemcpy(leaf_layout,leaf_layout_.data(),leaf_layout_.size()*sizeof(int),cudaMemcpyHostToDevice);
    cudaMemcpy(occlusionFreePointsCount,&initialPointCount,sizeof(int),cudaMemcpyHostToDevice);
    leaf_layout_size = leaf_layout_.size();
    int numBlocks = (int)ceil(numPoints/NUM_THREADS) + 1;

    // I am using way too many CUDA resources/registers, one block can't handle it, it has to be divided into multiple blocks

    double divider = 4.0f;
    int numberOfBlocks  = (int)ceil(numBlocks*divider);
    int numberOfThreads = (int)ceil(NUM_THREADS/divider);

    //std::cout<<"\nNumber of blocks to be used:"<<numberOfBlocks<<" the number of threads:"<<numberOfThreads;

    rayTraversalGPU<<<numberOfBlocks, numberOfThreads>>>(deviceX,deviceY,deviceZ,inverse_leaf_size,leaf_size,b_min,min_b,max_b,b_max,sensor_origin,maxAcc,leaf_layout,leaf_layout_size,divb_mul,occlusionFreePointsIndices,occlusionFreePointsEntropies,occlusionFreePointsCount,numPoints);

    cudaDeviceSynchronize();

    cudaError_t status =  cudaGetLastError();
    if (cudaSuccess != status)
      std::cout<<"\n CUDA Error!: error "<< cudaGetErrorString(status);

    std::vector<int> indices;
    std::vector<float> entropies;
    indices.resize(numPoints,0);
    entropies.resize(numPoints,0);
    cudaMemcpy(indices.data(),occlusionFreePointsIndices, numPoints*sizeof(int),cudaMemcpyDeviceToHost);
    cudaMemcpy(entropies.data(),occlusionFreePointsEntropies, numPoints*sizeof(float),cudaMemcpyDeviceToHost);


    // Entries with -one are occluded
    indices.erase(std::remove(indices.begin(), indices.end(), -1), indices.end());
    entropies.erase(std::remove(entropies.begin(), entropies.end(), -1.0), entropies.end());

    // Allocate enough space and copy the basics
    occlusionFreePoints.points.resize(indices.size());
    occlusionFreePoints.header   = inCloud->header;
    occlusionFreePoints.width    = static_cast<uint32_t>(indices.size());
    occlusionFreePoints.height   = 1;
    occlusionFreePoints.is_dense = inCloud->is_dense;
    occlusionFreePoints.sensor_orientation_ = inCloud->sensor_orientation_;
    occlusionFreePoints.sensor_origin_ = inCloud->sensor_origin_;
    // Iterate over each point
    for (size_t i = 0; i < indices.size (); ++i)
    {
        occlusionFreePoints.points[i] = inCloud->points[indices[i]];
    }
    entropyTot[0] =0 ;
    for (size_t i = 0; i < entropies.size (); ++i)
    {
        entropyTot[0] += entropies[i];
    }
//    std::cout<<"Total Entroby: "<<entropyTot[0]<<std::endl;

    free(x);
    free(y);
    free(z);
    cudaFree(deviceX);
    cudaFree(deviceY);
    cudaFree(deviceZ);
    cudaFree(inverse_leaf_size);
    cudaFree(leaf_size);
    cudaFree(b_min);
    cudaFree(sensor_origin);
    //cudaFree(occupiedP);
    //cudaFree(occupiedPPar);
    //cudaFree(unOccupiedPPar);
    cudaFree(maxAcc);
    cudaFree(divb_mul);
    cudaFree(leaf_layout);
    cudaFree(occlusionFreePointsIndices);
    cudaFree(occlusionFreePointsEntropies);
    cudaFree(occlusionFreePointsCount);
    cudaFree(min_b);
    cudaFree(max_b);
    cudaFree(b_max);
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float
pcl::VoxelGridOcclusionEstimationGPU::rayBoxIntersection (const Eigen::Vector4f& origin,
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
  return tmin;
}

float
pcl::VoxelGridOcclusionEstimationGPU::rayBoxIntersection (const Eigen::Vector4f& origin,
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
pcl::VoxelGridOcclusionEstimationGPU::rayTraversal (const Eigen::Vector3i& target_voxel,
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
pcl::VoxelGridOcclusionEstimationGPU::rayTraversal (std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> >& out_ray,
                                                         const Eigen::Vector3i& target_voxel,
                                                         const Eigen::Vector4f& origin,
                                                         const Eigen::Vector4f& direction,
                                                         const float t_min)
{
  // reserve space for the ray vector
  int reserve_size = div_b_.maxCoeff () * div_b_.maxCoeff ();
  out_ray.reserve (reserve_size);

  Eigen::Vector4f start = origin + t_min * direction;
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

  // the index of the cloud (-1 if empty)
  int index = -1;
  int result = 0;
  while ( (ijk[0]<= max_b_[0]+1) && (ijk[0]+1 >= min_b_[0]) &&
          (ijk[1]<= max_b_[1]+1) && (ijk[1]+1 >= min_b_[1]) &&
          (ijk[2]<= max_b_[2]+1) && (ijk[2]+1 >= min_b_[2]) )
  {

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
