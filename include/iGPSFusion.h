/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2021 Ze Yang
*/


#ifndef IGPSFUSION_H
#define IGPSFUSION_H

#include <iostream>
#include <thread>
#include <mutex>
#include <unistd.h>
#include <map>
#include <vector>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include<opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include<opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"

#include "Converter.h"
#include "LoopClosing.h"

using namespace std;

namespace ORB_SLAM3
{
    class LoopClosing;

class EdgeSE3Graph : public g2o::BaseBinaryEdge<6,g2o::SE3Quat,g2o::VertexSE3Expmap,g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeSE3Graph(double scale):_scale(scale){};

    virtual bool read(std::istream& is) override{};

    virtual bool write(std::ostream& os) const override{};

    virtual void computeError() override {
        const g2o::VertexSE3Expmap * v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        const g2o::VertexSE3Expmap * v2 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        g2o::SE3Quat wTi = v1->estimate();
        g2o::SE3Quat wTj = v2->estimate();
        g2o::SE3Quat iTj = wTi.inverse() * wTj;
        Eigen::Quaterniond iQj= iTj.rotation();
        Eigen::Vector3d iPj = iTj.translation();
        Eigen::Quaterniond obsQ = _measurement.rotation();
        Eigen::Vector3d obsP = _measurement.translation();
        _error[0] = _scale * obsP[0] - iPj[0];
        _error[1] = _scale * obsP[1] - iPj[1];
        _error[2] = _scale * obsP[2] - iPj[2];
        //std::cout<< "_error1 = " << _error[0] << "\t"<<  _error[1] << "\t"<<  _error[2] <<std::endl;

        Eigen::Quaterniond errorQ = obsQ.inverse() * iQj;
        _error[3] = 2 * errorQ.x();
        _error[4] = 2 * errorQ.y();
        _error[5] = 2 * errorQ.z();
        //std::cout<< "obsQ = " << obsQ <<std::endl;
        //std::cout<< "iQj = " << iQj <<std::endl;
        //std::cout<< "_error2 = " << _error[3] << "\t"<<  _error[4] << "\t"<<  _error[5] <<std::endl;

        //std::cout<< "_measurement = " << _measurement <<std::endl;

    }
    //virtual void linearizeOplus() override{};

private:
    double _scale;
};

class EdgeSE3iGPSFusion:public g2o::BaseUnaryEdge<3,g2o::Vector3D,g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeSE3iGPSFusion(){};
    //EdgeSE3iGPSFusion(double precision):_precision(precision){};

    virtual bool read(std::istream& is) override{};

    virtual bool write(std::ostream& os) const override{};

    virtual void computeError() override {
        const g2o::VertexSE3Expmap * v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        g2o::Vector3D obs(_measurement);
        g2o::Vector3D P3d = v1->estimate().translation();
        _error =  obs - P3d ;
        //std::cout<< "_error3 = " << _error.transpose() <<std::endl;
    }

    //virtual void linearizeOplus() override{
    //    const g2o::VertexSE3Expmap * v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    //    g2o::Vector3D P3d = v1->estimate().translation();
    //    _jacobianOplusXi.block<3,3>(0,0) = Eigen::Matrix3d::Zero();
    //    _jacobianOplusXi.block<3,3>(0,3) = -Eigen::Matrix3d::Identity();
    //};

private:
    //double _precision;
};

class EdgeSim3:public g2o::BaseUnaryEdge<3,g2o::Vector3D,g2o::VertexSim3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeSim3(g2o::Vector3D pos):_pos3d(pos){};

    virtual bool read(std::istream& is) override{};

    virtual bool write(std::ostream& os) const override{};

    virtual void computeError() override {
        const g2o::VertexSim3Expmap * v1 = static_cast<const g2o::VertexSim3Expmap*>(_vertices[0]);
        g2o::Vector3D obs(_measurement);
        g2o::Vector3D map = v1->estimate().map(_pos3d);
        _error =  obs - map ;
    }

    virtual void linearizeOplus() override{
      const g2o::VertexSim3Expmap * v1 = static_cast<const g2o::VertexSim3Expmap*>(_vertices[0]);
      g2o::Vector3D p = v1->estimate().map(_pos3d);

      Eigen::Matrix3d p_skew;
      p_skew << 0.0, -p[2], p[1], p[2], 0.0, -p[0], -p[1], p[0], 0.0;

      _jacobianOplusXi.block<3,3>(0,0) = p_skew;
      _jacobianOplusXi.block<3,3>(0,3) = -Eigen::Matrix3d::Identity();
      _jacobianOplusXi.block<3,1>(0,6) = -p;
    };

private:
    g2o::Vector3D _pos3d;
};

class iGPSFusion
{
public:
    iGPSFusion();
    void RequestFinish();
    bool isFinished();

    void optimize();
    void SetLoopCloser(LoopClosing* mptLoopCloser);
    void inputiGPS(double t, cv::Point3f p3D);
    void inputVO(double t, cv::Mat pose);
    bool getFusionResult(Eigen::Vector3d & P,Eigen::Quaterniond & Q);

    
protected:
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    bool mbStopped;
    LoopClosing* mpLoopClosing;
    std::mutex mMutexFinish;
    std::mutex mMutexStop;
private:
    Eigen::Vector3d last_p;
    Eigen::Quaterniond last_q;
    Eigen::Matrix4d iGPS_T_VO;
    std::map<double, std::vector<double>> VOPoseMap;
    std::map<double, cv::Point3f> iGPSPositionMap;
    std::map<double, std::vector<double>> mmFusionPoseMap;
    double mScale,mLastScale;
    bool mbfusionFlag,mbScaleFlag;
    std::mutex mMutexiGPS,m_PoseMap;
};

} //namespace ORB_SLAM

#endif //
