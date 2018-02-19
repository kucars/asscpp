#include <stdio.h>

// includes CUDA Runtime
#include <cuda_runtime.h>

int main(int argc, char *argv[])
{
    int nDevices;
    cudaGetDeviceCount(&nDevices);
    for (int i = 0; i < nDevices; i++)
    {
        cudaDeviceProp prop;
        cudaGetDeviceProperties(&prop, i);
        printf("Device Number: %d\n", i);
        printf("  Device name: %s\n", prop.name);
        printf("  Memory Clock Rate (KHz): %d\n", prop.memoryClockRate);
        printf("  Memory Bus Width (bits): %d\n", prop.memoryBusWidth);
        printf("  Peak Memory Bandwidth (GB/s): %f\n", 2.0*prop.memoryClockRate*(prop.memoryBusWidth/8)/1.0e6);
        printf("  Max Threads per block:%d\n",prop.maxThreadsPerBlock);
        printf("  Max Threads dimensions x:%d\n",prop.maxThreadsDim[0]);
        printf("  Max Threads dimensions y:%d\n",prop.maxThreadsDim[1]);
        printf("  Max Threads dimensions z:%d\n",prop.maxThreadsDim[2]);
        printf("  Max Grid size x:%d\n",prop.maxGridSize[0]);
        printf("  Max Grid size y:%d\n",prop.maxGridSize[1]);
        printf("  Max Grid size z:%d\n",prop.maxGridSize[2]);
    }
}
