/*
 * TVHist.hpp
 *
 *  Created on: Jan 22, 2014
 *      Author: s.zagoruyko
 */

#ifndef TVHIST_HPP_
#define TVHIST_HPP_

#include <memory>
#include <Eigen/Geometry>

#include <common.hpp>

namespace atlas {
namespace depthfusion {

class TVHist
{
public:
	typedef std::shared_ptr<TVHist> Ptr;
//	typedef cl_uchar8 Voxel;
    enum {BINS = 8};
    struct Voxel {
        cl_uchar hist[BINS];
    };

	typedef Eigen::Vector3i Vector3i;
	typedef Eigen::Vector3f Vector3f;

    TVHist (cl::CommandQueue queue, const Vector3i& resolution, const Vector3f& dims, int rows, int cols, bool verbose = true,
            float lambda = 0.25f, float delta = 0.05f, float eta = 0.05f, float tau = 0.16f, float theta = 0.02f,
            float empty_weight = 1.0f, float occl_weight = 1.0f);

    void
    reset ();

    void
    setResolution (const Vector3i& res) { resolution = res; }

    void
    setDims (const Vector3f& dims) { this->dims = dims; }

    void
    setLambda (float lambda) { this->lambda = lambda; }

    void
    setDelta (float delta) { this->delta = delta; }

    void
    setEta (float eta) { this->eta = eta; }

    void
    setTau (float tau) { this->tau = tau; }

    void
    setTheta (float theta) { this->theta = theta; }

    void
    setEmptyWeight (float weight) { empty_weight = weight; }

    void
    setOcclWeight (float weight) { occl_weight = weight; }

    void
    exportData (cl::Buffer& data);

    void
    exportData (std::vector<float>& data);

    void
    upscaleData (const cl::Buffer& coarseData);

    void
    upscaleData (const cl::Buffer& coarseData, cl_float4 cubeShift, cl_float4 cubeScale);

    void
    addDepthToVolume (const std::vector<cl_uchar>& raw_depth, const Eigen::Affine3f& pose, cl_float4 intrinsics, cl_float2& ksi, int layers);

    void
    addDepthToVolume (const std::vector<cl_ushort>& raw_depth, const Eigen::Affine3f& pose, cl_float4 intrinsics);

    /* be carefull using that
     *
     */
    void
    addDepthToVolume (const cl_ushort* raw_depth, const Eigen::Affine3f& pose, cl_float4 intrinsics);

    std::vector<cl_float4>
    projectToWorld (std::vector<cl_float4>& rgb, std::vector<cl_uchar>& raw_depth, const Eigen::Affine3f& pose, cl_float4 intrinsics);

    void
    optimize ();

    void
    convertInputData (cl::Buffer& data);

    void
    resetPrimalDual ();

    int
    getBinsCount () const { return sizeof(Voxel); }

    void
    showTSDF();

    void
    showPrimal();

    void
    checkValues(float minVal, float maxVal, cl_int3& minXYZ, cl_int3& maxXYZ, cl_int3 startXYZ, cl_int3 endXYZ);

//    friend class TVHistTest;
    cl::Buffer tsdf, inv_depth, world_cloud, depth;
    cl::Buffer up;
protected:
    void addDepth (const Eigen::Affine3f& pose, cl_float4 intrinsics);

protected:
//    cl::Buffer tsdf, inv_depth, world_cloud, depth;
//    cl::Buffer u, p;

    cl::CommandQueue queue;
    Vector3i resolution;
    Vector3f dims;
    int rows, cols;
    bool verbose;
    float lambda, delta, eta, tau, theta, empty_weight, occl_weight;
    cl::Program program;
    Eigen::Affine3f ref_pose;
};


struct RGBDepth {
    std::vector<cl_float4> rgb;
    std::vector<cl_uchar> depth;
    Eigen::Affine3f pose;

    void resize (size_t size)
    {
        rgb.resize(size);
        depth.resize(size);
    }
};

}
}

#endif /* TVHIST_HPP_ */
