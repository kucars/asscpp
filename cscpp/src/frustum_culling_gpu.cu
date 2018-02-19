/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 */

#include <cscpp/frustum_culling_gpu.h>
#include <pcl/common/io.h>
#include <vector>


///////////////////////////////////////////////////////////////////////////////
pcl::FrustumCullingGPU::~FrustumCullingGPU()
{
    free(x);
    free(y);
    free(z);
    cudaFree(deviceX);
    cudaFree(deviceY);
    cudaFree(deviceZ);
    cudaFree(device_pl_t);
    cudaFree(device_pl_b);
    cudaFree(device_pl_l);
    cudaFree(device_pl_r);
    cudaFree(device_pl_n);
    cudaFree(device_pl_f);
    cudaFree(deviceIndices);
    cudaFree(deviceIndicesCount);
}

void pcl::FrustumCullingGPU::initializeGPUPointData(const PointCloudConstPtr &cloud)
{
    setInputCloud(cloud);
    if(input_->points.size() == 0)
        return;
    numPoints = input_->points.size();
    int size  = numPoints*sizeof(double);
    std::cout<<"\nCreated an array of size:"<<input_->points.size();

    cudaMalloc((void**)&device_pl_n, 4*sizeof(float));
    cudaMalloc((void**)&device_pl_f, 4*sizeof(float));
    cudaMalloc((void**)&device_pl_t, 4*sizeof(float));
    cudaMalloc((void**)&device_pl_b, 4*sizeof(float));
    cudaMalloc((void**)&device_pl_l, 4*sizeof(float));
    cudaMalloc((void**)&device_pl_r, 4*sizeof(float));

    cudaMalloc((void**)&deviceIndices, numPoints*sizeof(int));
    cudaMalloc((void**)&deviceIndicesCount,sizeof(int));

    cudaMalloc((void**)&deviceX, size);
    cudaMalloc((void**)&deviceY, size);
    cudaMalloc((void**)&deviceZ, size);

    x = (double*)malloc(size);
    y = (double*)malloc(size);
    z = (double*)malloc(size);

    for(int i=0;i<input_->points.size();i++)
    {
        x[i] = input_->points[i].x;
        y[i] = input_->points[i].y;
        z[i] = input_->points[i].z;
    }

    int initialCount = 0;

    cudaMemcpy(deviceIndicesCount,&initialCount,sizeof(int),cudaMemcpyHostToDevice);
    cudaMemcpy(deviceX,x,size,cudaMemcpyHostToDevice);
    cudaMemcpy(deviceY,y,size,cudaMemcpyHostToDevice);
    cudaMemcpy(deviceZ,z,size,cudaMemcpyHostToDevice);
}

void pcl::FrustumCullingGPU::applyFilter (PointCloud& output)
{
    std::vector<int> indices;
    if (keep_organized_)
    {
        bool temp = extract_removed_indices_;
        extract_removed_indices_ = true;
        applyFilter (indices);
        extract_removed_indices_ = temp;
        copyPointCloud (*input_, output);

        for (size_t rii = 0; rii < removed_indices_->size (); ++rii)
        {
            pcl::PointXYZ &pt_to_remove = output.at ((*removed_indices_)[rii]);
            pt_to_remove.x = pt_to_remove.y = pt_to_remove.z = user_filter_value_;
            if (!pcl_isfinite (user_filter_value_))
                output.is_dense = false;
        }
    }
    else
    {
        output.is_dense = true;
        applyFilter (indices);
        copyPointCloud (*input_, indices, output);
    }
}

__global__ void point_inside_fov_check(double *x, double*y, double*z,float * pl_t,float * pl_b,float * pl_r,float * pl_l,float * pl_n,float * pl_f,int *indices, int* indices_count,int numPoints)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    bool negative = false;
    if(i<numPoints)
    {
        if(i==0)
            indices_count[0] = 0;
        double pt_dot_pl_t = x[i]*pl_t[0] + y[i]*pl_t[1] + z[i]*pl_t[2] + 1.0*pl_t[3];
        double pt_dot_pl_b = x[i]*pl_b[0] + y[i]*pl_b[1] + z[i]*pl_b[2] + 1.0*pl_b[3];
        double pt_dot_pl_r = x[i]*pl_r[0] + y[i]*pl_r[1] + z[i]*pl_r[2] + 1.0*pl_r[3];
        double pt_dot_pl_l = x[i]*pl_l[0] + y[i]*pl_l[1] + z[i]*pl_l[2] + 1.0*pl_l[3];
        double pt_dot_pl_n = x[i]*pl_n[0] + y[i]*pl_n[1] + z[i]*pl_n[2] + 1.0*pl_n[3];
        double pt_dot_pl_f = x[i]*pl_f[0] + y[i]*pl_f[1] + z[i]*pl_f[2] + 1.0*pl_f[3];

        if( ( (pt_dot_pl_t<=0) && (pt_dot_pl_b<=0) && (pt_dot_pl_r<=0) && (pt_dot_pl_l<=0) && (pt_dot_pl_n<=0) && (pt_dot_pl_f<=0)) ^ negative)
        {
            indices[i] = i;
            //indices[indices_count[0]] = 0;
            atomicAdd(&indices_count[0], 1);
        }
        else
        {
            indices[i] = 0;
        }
    }
}

void pcl::FrustumCullingGPU::applyFilter (std::vector<int> &indices)
{
    Eigen::Vector4f pl_n; // near plane
    Eigen::Vector4f pl_f; // far plane
    Eigen::Vector4f pl_t; // top plane
    Eigen::Vector4f pl_b; // bottom plane
    Eigen::Vector4f pl_r; // right plane
    Eigen::Vector4f pl_l; // left plane

    Eigen::Vector3f view = camera_pose_.block (0, 0, 3, 1);    // view vector for the camera  - first column of the rotation matrix
    Eigen::Vector3f up = camera_pose_.block (0, 1, 3, 1);      // up vector for the camera    - second column of the rotation matix
    Eigen::Vector3f right = camera_pose_.block (0, 2, 3, 1);   // right vector for the camera - third column of the rotation matrix
    Eigen::Vector3f T = camera_pose_.block (0, 3, 3, 1);       // The (X, Y, Z) position of the camera w.r.t origin


    float vfov_rad = float (vfov_ * M_PI / 180); // degrees to radians
    float hfov_rad = float (hfov_ * M_PI / 180); // degrees to radians

    float np_h = float (2 * tan (vfov_rad / 2) * np_dist_);  // near plane height
    float np_w = float (2 * tan (hfov_rad / 2) * np_dist_);  // near plane width

    float fp_h = float (2 * tan (vfov_rad / 2) * fp_dist_);    // far plane height
    float fp_w = float (2 * tan (hfov_rad / 2) * fp_dist_);    // far plane width

    Eigen::Vector3f fp_c (T + view * fp_dist_);                 // far plane center
    fp_tl =(fp_c + (up * fp_h / 2) - (right * fp_w / 2));  // Top left corner of the far plane
    fp_tr =(fp_c + (up * fp_h / 2) + (right * fp_w / 2));  // Top right corner of the far plane
    fp_bl =(fp_c - (up * fp_h / 2) - (right * fp_w / 2));  // Bottom left corner of the far plane
    fp_br =(fp_c - (up * fp_h / 2) + (right * fp_w / 2));  // Bottom right corner of the far plane

    Eigen::Vector3f np_c (T + view * np_dist_);                   // near plane center
    np_tl = np_c + (up * np_h/2) - (right * np_w/2); // Top left corner of the near plane
    np_tr =(np_c + (up * np_h / 2) + (right * np_w / 2));   // Top right corner of the near plane
    np_bl =(np_c - (up * np_h / 2) - (right * np_w / 2));   // Bottom left corner of the near plane
    np_br =(np_c - (up * np_h / 2) + (right * np_w / 2));   // Bottom right corner of the near plane

    pl_f.block (0, 0, 3, 1).matrix () = (fp_bl - fp_br).cross (fp_tr - fp_br);   // Far plane equation - cross product of the
    pl_f (3) = -fp_c.dot (pl_f.block (0, 0, 3, 1));                    // perpendicular edges of the far plane

    pl_n.block (0, 0, 3, 1).matrix () = (np_tr - np_br).cross (np_bl - np_br);   // Near plane equation - cross product of the
    pl_n (3) = -np_c.dot (pl_n.block (0, 0, 3, 1));                    // perpendicular edges of the far plane

    Eigen::Vector3f a (fp_bl - T);    // Vector connecting the camera and far plane bottom left
    Eigen::Vector3f b (fp_br - T);    // Vector connecting the camera and far plane bottom right
    Eigen::Vector3f c (fp_tr - T);    // Vector connecting the camera and far plane top right
    Eigen::Vector3f d (fp_tl - T);    // Vector connecting the camera and far plane top left

    //                   Frustum and the vectors a, b, c and d. T is the position of the camera
    //                             _________
    //                           /|       . |
    //                       d  / |   c .   |
    //                         /  | __._____|
    //                        /  /  .      .
    //                 a <---/-/  .    .
    //                      / / .   .  b
    //                     /   .
    //                     .
    //                   T
    //

    pl_r.block (0, 0, 3, 1).matrix () = b.cross (c);
    pl_l.block (0, 0, 3, 1).matrix () = d.cross (a);
    pl_t.block (0, 0, 3, 1).matrix () = c.cross (d);
    pl_b.block (0, 0, 3, 1).matrix () = a.cross (b);

    pl_r (3) = -T.dot (pl_r.block (0, 0, 3, 1));
    pl_l (3) = -T.dot (pl_l.block (0, 0, 3, 1));
    pl_t (3) = -T.dot (pl_t.block (0, 0, 3, 1));
    pl_b (3) = -T.dot (pl_b.block (0, 0, 3, 1));

    if (extract_removed_indices_)
    {
        removed_indices_->resize (indices_->size ());
    }
    int size = 4*sizeof(float);
    int indicesCount=0;
    float test[4];
    cudaMemcpy(device_pl_t,pl_t.data(),size,cudaMemcpyHostToDevice);
    cudaMemcpy(device_pl_b,pl_b.data(),size,cudaMemcpyHostToDevice);
    cudaMemcpy(device_pl_r,pl_r.data(),size,cudaMemcpyHostToDevice);
    cudaMemcpy(device_pl_l,pl_l.data(),size,cudaMemcpyHostToDevice);
    cudaMemcpy(device_pl_n,pl_n.data(),size,cudaMemcpyHostToDevice);
    cudaMemcpy(device_pl_f,pl_f.data(),size,cudaMemcpyHostToDevice);

    cudaMemcpy(test,device_pl_t,size,cudaMemcpyDeviceToHost);
    int numBlocks = (int)ceil(numPoints/NUM_THREADS) + 1;

    point_inside_fov_check<<<numBlocks, NUM_THREADS>>>(deviceX,deviceY,deviceZ,device_pl_t,device_pl_b,device_pl_r,device_pl_l,device_pl_n,device_pl_f,deviceIndices,deviceIndicesCount,numPoints);

    indices.resize(numPoints,0);
    cudaMemcpy(&indicesCount,deviceIndicesCount, sizeof(int),cudaMemcpyDeviceToHost);
    cudaMemcpy(indices.data(),deviceIndices, numPoints*sizeof(int),cudaMemcpyDeviceToHost);

    // Entries with zeros are outside the frustum
    indices.erase(std::remove(indices.begin(), indices.end(), 0), indices.end());
    // TODO: maybe we need to keep te removed indicies ?
}
