/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORB_SLAM3_OPTIMIZABLETYPES_H
#define ORB_SLAM3_OPTIMIZABLETYPES_H

#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include <Thirdparty/g2o/g2o/types/types_six_dof_expmap.h>
#include <Thirdparty/g2o/g2o/types/sim3.h>
#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"

#include <Eigen/Geometry>
#include <include/CameraModels/GeometricCamera.h>

namespace ORB_SLAM3 {
class  EdgeSE3ProjectXYZOnlyPose: public  g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZOnlyPose(){}

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs-pCamera->project(v1->estimate().map(Xw));
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        return (v1->estimate().map(Xw))(2)>0.0;
    }


    virtual void linearizeOplus();

    Eigen::Vector3d Xw;
    GeometricCamera* pCamera;
};

class  EdgeSE3ProjectXYZOnlyPoseToBody: public  g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZOnlyPoseToBody(){}

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs-pCamera->project((mTrl * v1->estimate()).map(Xw));
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        return ((mTrl * v1->estimate()).map(Xw))(2)>0.0;
    }


    virtual void linearizeOplus();

    Eigen::Vector3d Xw;
    GeometricCamera* pCamera;

    g2o::SE3Quat mTrl;
};

class  EdgeSE3ProjectXYZ: public  g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZ();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs-pCamera->project(v1->estimate().map(v2->estimate()));
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        return ((v1->estimate().map(v2->estimate()))(2)>0.0);
    }

    virtual void linearizeOplus();

    GeometricCamera* pCamera;
};

/*
class  EdgeSE3ProjectXYZTd: public g2o::BaseMultiEdge<2,Eigen::Vector2d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZTd(cv::Point2f kpUnSpeed, double td):_speed(kpUnSpeed),_td(td){}

    virtual bool read(std::istream& is) override{}

    virtual bool write(std::ostream& os) const override{}

    virtual void computeError()  override{
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        const VertexTime* v3 = static_cast<const VertexTime*>(_vertices[2]);
        Eigen::Vector2d speed(_speed.x,_speed.y);
        _measurement = _measurement - (v3->estimate() - _td) * speed;
        Eigen::Vector2d obs(_measurement);
        _error = obs-pCamera->project(v1->estimate().map(v2->estimate()));
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        return ((v1->estimate().map(v2->estimate()))(2)>0.0);
    }

    //virtual void linearizeOplus();
    cv::Point2f _speed;
    double _td;
    GeometricCamera* pCamera;
};
*/
class  EdgeSE3ProjectXYZToBody: public  g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZToBody();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs-pCamera->project((mTrl * v1->estimate()).map(v2->estimate()));
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        return ((mTrl * v1->estimate()).map(v2->estimate()))(2)>0.0;
    }

    virtual void linearizeOplus();

    GeometricCamera* pCamera;
    g2o::SE3Quat mTrl;
};

// time offset vertex

class VertexSim3Expmap : public g2o::BaseVertex<7, g2o::Sim3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexSim3Expmap();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setToOriginImpl() {
        _estimate = g2o::Sim3();
    }

    virtual void oplusImpl(const double* update_)
    {
        Eigen::Map<g2o::Vector7d> update(const_cast<double*>(update_));

        if (_fix_scale)
            update[6] = 0;

        g2o::Sim3 s(update);
        setEstimate(s*estimate());
    }

    GeometricCamera* pCamera1, *pCamera2;

    bool _fix_scale;
};


class EdgeSim3ProjectXYZ : public  g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, ORB_SLAM3::VertexSim3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSim3ProjectXYZ();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    void computeError()
    {
        const ORB_SLAM3::VertexSim3Expmap* v1 = static_cast<const ORB_SLAM3::VertexSim3Expmap*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

        Eigen::Vector2d obs(_measurement);
        _error = obs-v1->pCamera1->project(v1->estimate().map(v2->estimate()));
    }

    // virtual void linearizeOplus();

};

class EdgeInverseSim3ProjectXYZ : public  g2o::BaseBinaryEdge<2, Eigen::Vector2d,  g2o::VertexSBAPointXYZ, VertexSim3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeInverseSim3ProjectXYZ();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    void computeError()
    {
        const ORB_SLAM3::VertexSim3Expmap* v1 = static_cast<const ORB_SLAM3::VertexSim3Expmap*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

        Eigen::Vector2d obs(_measurement);
        _error = obs-v1->pCamera2->project((v1->estimate().inverse().map(v2->estimate())));
    }

    // virtual void linearizeOplus();

};

class EdgeSE3Fusion:public g2o::BaseUnaryEdge<3,Eigen::Vector3d,g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeSE3Fusion(){};

    virtual bool read(std::istream& is) override{};

    virtual bool write(std::ostream& os) const override{};

    virtual void computeError() override {
        const g2o::VertexSE3Expmap * v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector3d obs(_measurement);
        Eigen::Matrix3d r = v1->estimate().rotation().toRotationMatrix();
        Eigen::Vector3d t = v1->estimate().translation();
        Eigen::Vector3d t_inverse = - r.transpose() * t;
        //std::cout<< "obs = " << obs <<std::endl;
        //std::cout<< "t_inverse = " << t_inverse <<std::endl;
        _error =  obs - t_inverse ;
    }
private:
    //double _precision;
};

class EdgeSE36DoFFusion:public g2o::BaseUnaryEdge<6,Eigen::Matrix<double,6,1>,g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeSE36DoFFusion(){};

    virtual bool read(std::istream& is) override{};

    virtual bool write(std::ostream& os) const override{};

    virtual void computeError() override {
        const g2o::VertexSE3Expmap * v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        Eigen::Matrix<double,6,1> obs(_measurement);
        Eigen::Matrix3d r = v1->estimate().rotation().toRotationMatrix();
        Eigen::Vector3d t = v1->estimate().translation();
        Eigen::Vector3d t_inverse = - r.transpose() * t;

        Eigen::Quaterniond q1(r.transpose());
        //std::cout<< "obs = " << obs <<std::endl;
        //std::cout<< "t_inverse = " << t_inverse <<std::endl;
        _error[0] =  obs[0] - t_inverse[0] ;
        _error[1] =  obs[1] - t_inverse[1] ;
        _error[2] =  obs[2] - t_inverse[2] ;

        Eigen::Matrix3d R,Rx,Ry,Rz,pkR,RR;
        double a= obs[3], b= obs[4], c= obs[5];
        Rx<< 1, 0, 0, 0, cos(a), -sin(a), 0, sin(a), cos(a);
        Ry<< cos(b), 0, sin(b), 0, 1, 0, -sin(b), 0, cos(b);
        Rz<< cos(c), -sin(c), 0, sin(c), cos(c), 0, 0, 0, 1;
        R = Rz * Ry *Rx;

        Eigen::Quaterniond q2(R);
        Eigen::Quaterniond q = q2.inverse()*q1;
        //if(q.x()>0.05||q.y()>0.05||q.z()>0.05)
        //{
        //    std::cout<< "q1 = " << q1<<std::endl;
        //    std::cout<< "q2 = " << q2<<std::endl;
        //    std::cout<< "q = " << q<<std::endl;
        //}
        _error[3] =  q.x();
        _error[4] =  q.y();
        _error[5] =  q.z();
    }
private:
    //double _precision;
};

}

#endif //ORB_SLAM3_OPTIMIZABLETYPES_H
