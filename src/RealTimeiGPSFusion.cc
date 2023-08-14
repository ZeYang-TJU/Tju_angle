//
// Created by Alex Xiangchen Liu on 2022/4/20.
//
#include "RealTimeiGPSFusion.h"

namespace ORB_SLAM3
{

    RealTimeiGPSFusion::RealTimeiGPSFusion():mbScaleFlag(false),mScale(1.0),mLastScale(1.0),mbFinishRequested(false), mbFinished(true),mLast_time(0.0),
                             mbStopped(false)
    {
        mbfusionFlag = false;
        last_p = Eigen::Vector3d(0.0,0.0,0.0);
        last_q = Eigen::Quaterniond(0.0,0.0,0.0,0.0);
        iGPS_T_VO = Eigen::Matrix4d::Identity();
        mEstimatedTransformation= Eigen::Matrix4d::Identity();
        std::cout<< "Run Real Time Frame-to-Frame Monocular-iGPS thread"<<std::endl;
    }

    void RealTimeiGPSFusion::inputiGPS(double t, cv::Point3f p3D)
    {
        iGPSPositionMap[t] = p3D;
        mLast_time = t;
        mMutexiGPS.lock();
        mbfusionFlag = true;
        mMutexiGPS.unlock();
    }

    void RealTimeiGPSFusion::RequestFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool RealTimeiGPSFusion::isFinished()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

    void RealTimeiGPSFusion::SetFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
        unique_lock<mutex> lock2(mMutexStop);
        mbStopped = true;
    }

    void RealTimeiGPSFusion::SetTracker(Tracking *mptTracker)
    {
        mpTracker = mptTracker;
    }

    void RealTimeiGPSFusion::SetLoopCloser(LoopClosing *mptLoopCloser)
    {
        mpLoopCloser = mptLoopCloser;
    }

    void RealTimeiGPSFusion::inputVO(double t, cv::Mat Tcw)
    {
        if (Tcw.empty())
            return;
        else
        {
            m_PoseMap.lock();

            if(VOPoseMap.size())
            {
                auto iterator = VOPoseMap.rbegin();
                auto iteratoriGPS = iGPSPositionMap.find(iterator->first);
                if(iteratoriGPS == iGPSPositionMap.end())
                {
                    auto iterator_next =  VOPoseMap.find(mLast_time);
                    if(iterator_next != VOPoseMap.end())
                    {
                        Eigen::Quaterniond localQ(iterator->second[3],iterator->second[4],iterator->second[5],iterator->second[6]);
                        Eigen::Vector3d localP = Eigen::Vector3d(iterator->second[0],iterator->second[1],iterator->second[2]);
                        localQ = Eigen::Quaterniond(iGPS_T_VO.block<3,3>(0,0)) * localQ;
                        localP = mLastScale * iGPS_T_VO.block<3,3>(0,0) * localP + iGPS_T_VO.block<3,1>(0,3);

                        Eigen::Quaterniond localQ_next (iterator_next->second[3],iterator_next->second[4],iterator_next->second[5],iterator_next->second[6]);
                        Eigen::Vector3d localP_next = Eigen::Vector3d(iterator_next->second[0],iterator_next->second[1],iterator_next->second[2]);
                        localQ_next = Eigen::Quaterniond(iGPS_T_VO.block<3,3>(0,0)) * localQ_next;
                        localP_next = mLastScale * iGPS_T_VO.block<3,3>(0,0) * localP_next + iGPS_T_VO.block<3,1>(0,3);

                        Eigen::Matrix4d T = Eigen::Matrix4d::Identity(); Eigen::Matrix4d T1= Eigen::Matrix4d::Identity(); Eigen::Matrix4d T2 = Eigen::Matrix4d::Identity();
                        T1.block<3,3>(0,0) = localQ.toRotationMatrix();
                        T1.block<3,1>(0,3) = localP;

                        T2.block<3,3>(0,0) = localQ_next.toRotationMatrix();
                        T2.block<3,1>(0,3) = localP_next;
                        T = T1* T2.inverse();

                        auto iter = mmFusionPoseMap.find(iterator->first);
                        auto iter_next = mmFusionPoseMap.find(iterator_next->first);

                        Eigen::Quaterniond GlobalQ(iter_next->second[3],iter_next->second[4],iter_next->second[5],iter_next->second[6]);
                        Eigen::Vector3d GlobalP = Eigen::Vector3d(iter_next->second[0],iter_next->second[1],iter_next->second[2]);

                        Eigen::Matrix4d GlobalT = Eigen::Matrix4d::Identity();
                        GlobalT.block<3,3>(0,0) = GlobalQ.toRotationMatrix();
                        GlobalT.block<3,1>(0,3) = GlobalP;
                        GlobalT = T * GlobalT ;

                        Eigen::Vector3d testP = GlobalT.block<3,1>(0,3);

                        //ofstream f;
                        //f.open("./RealTimeResult.txt",ios::app);
                        //f << fixed;
                        //f << iter->first  << "," <<  setprecision(9) << ( testP.x() + iter->second[0])/2 << "," << (testP.y() +iter->second[1]) /2 << "," << (testP.z() +iter->second[2]) /2 <<endl;
                        //f << iter->first  << "," <<  setprecision(9) << estimated1.x() << "," << estimated1.y() << "," << estimated1.z()  <<endl;
                        //f << iter->first  << "," <<  setprecision(9) << iter->second[0]<< "," << iter->second[1]<< "," << iter->second[2] <<endl;
                        //f << iter->first  << "," <<  setprecision(9) << testP.x()  << "," << testP.y()  << "," << testP.z() <<endl;

                        //f << iter->first  << "," <<  setprecision(9) << iter->second[0] << "," << iter->second[1] << "," << iter->second[2] << "," << iter->second[3] << "," << iter->second[4] << "," << iter->second[5] << ","  << iter->second[6]  <<endl;
                        //f.close();
                        //cout<< "Fail to track iGPS, Global localization result2 = " << iter->first << "," << testP.x() << "," << testP.y() << "," << testP.z() <<endl;
                        //cout<< "Fail to track iGPS, Global localization result3 = " << iter->first << "," << iter->second[0] << "," << iter->second[1] << "," << iter->second[2] <<endl;
                    }
                }
            }

            Eigen::Quaterniond localQ(Converter::toMatrix3d(Tcw.rowRange(0,3).colRange(0,3)));
            Eigen::Vector3d localPinv = Converter::toVector3d(Tcw.rowRange(0,3).col(3));
            Eigen::Vector3d localP = -(localQ.inverse()*localPinv);
            Eigen::Quaterniond FusionQ = Eigen::Quaterniond(iGPS_T_VO.block<3,3>(0,0)) * localQ;
            Eigen::Vector3d FusionP = mLastScale * iGPS_T_VO.block<3,3>(0,0) * localP + iGPS_T_VO.block<3,1>(0,3);

            if(mScale != 1.0)
                mLastScale = mScale;

            std::vector<double> localPose{localP.x(),localP.y(),localP.z(),
                                          localQ.w(),localQ.x(),localQ.y(),localQ.z()};
            VOPoseMap[t] = localPose;

            std::vector<double> FusionPose{ FusionP.x(),FusionP.y(),FusionP.z(),
                                            FusionQ.w(),FusionQ.x(),FusionQ.y(),FusionQ.z()};

            mmFusionPoseMap[t] = FusionPose;

            //if scale is estimated successfully
            if(mbScaleFlag && mScale != 1.0)
            {
                mScale = 1;
                mmFusionPoseMap.clear();
                std::cout<< "iGPS_T_VO = " << iGPS_T_VO <<std::endl;
                std::cout<< "mLastScale = " << mLastScale <<std::endl;
                for(auto iterator = VOPoseMap.begin(); iterator != VOPoseMap.end();iterator++)
                {
                    Eigen::Quaterniond localQ(iterator->second[3],iterator->second[4],iterator->second[5],iterator->second[6]);
                    Eigen::Vector3d localP = Eigen::Vector3d(iterator->second[0],iterator->second[1],iterator->second[2]);

                    FusionQ = Eigen::Quaterniond(iGPS_T_VO.block<3,3>(0,0)) * localQ;
                    FusionP = mLastScale * iGPS_T_VO.block<3,3>(0,0) * localP + iGPS_T_VO.block<3,1>(0,3);

                    std::vector<double> FusionPose{ FusionP.x(),FusionP.y(),FusionP.z(),
                                                    FusionQ.w(),FusionQ.x(),FusionQ.y(),FusionQ.z()};
                    mmFusionPoseMap[iterator->first] = FusionPose;
                }
            }

            //last_p = FusionP;
            //last_q = FusionQ;
            m_PoseMap.unlock();
        }
    }

    bool RealTimeiGPSFusion::getFusionResult(Eigen::Vector3d & P, Eigen::Quaterniond & Q)
    {
        P = last_p;
        Q = last_q;
    }

    void RealTimeiGPSFusion::optimize()
    {
        mbFinished = false;

        while(1)
        {
            bool bFusionSolution =false;

            mMutexiGPS.lock();   //lock iGPS Flag
            if(mbfusionFlag)
            {
                bFusionSolution = true;
                mbfusionFlag = false;
            }
            mMutexiGPS.unlock();

            if(bFusionSolution)
            {
                int num = iGPSPositionMap.size();
                //cout << "num " << num << endl;
                if(num <20)
                    continue;

                if(mbScaleFlag)
                {
                    //Pose Graph Optimize
                    g2o::SparseOptimizer optimizer;
                    g2o::BlockSolverX::LinearSolverType * linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
                    auto * solver_ptr = new g2o::BlockSolverX(linearSolver);
                    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
                    optimizer.setAlgorithm(solver);
                    optimizer.setVerbose(true);
                    m_PoseMap.lock();

                    int index = 0;
                    int length = mmFusionPoseMap.size();
                    int iFixLength = 100;
                    for(auto iterator = mmFusionPoseMap.begin(); iterator != mmFusionPoseMap.end();iterator++)
                    {
                        if(length - iFixLength>0)
                        {
                            length--;
                            continue;
                        }
                        Eigen::Quaterniond localQ(iterator->second[3],iterator->second[4],iterator->second[5],iterator->second[6]);
                        Eigen::Vector3d localP = Eigen::Vector3d(iterator->second[0],iterator->second[1],iterator->second[2]);
                        auto *v = new g2o::VertexSE3Expmap();
                        v->setId(index);
                        v->setFixed(false);
                        v->setEstimate(g2o::SE3Quat(localQ,localP));
                        v->setMarginalized(false);
                        optimizer.addVertex(v);
                        index++;
                    }

                    index = 0;
                    length = mmFusionPoseMap.size();
                    for(auto iterator = VOPoseMap.begin(); iterator != VOPoseMap.end();iterator++)
                    {
                        if(length-iFixLength>0)
                        {
                            length--;
                            continue;
                        }
                        auto iteratorNext = iterator;
                        iteratorNext++;
                        if(iteratorNext != VOPoseMap.end())
                        {
                            Eigen::Quaterniond localQ(iterator->second[3],iterator->second[4],iterator->second[5],iterator->second[6]);
                            Eigen::Vector3d localP = Eigen::Vector3d(iterator->second[0],iterator->second[1],iterator->second[2]);
                            g2o::SE3Quat wTi = g2o::SE3Quat (localQ,localP);
                            localQ = Eigen::Quaterniond(iteratorNext->second[3],iteratorNext->second[4],iteratorNext->second[5],iteratorNext->second[6]);
                            localP = Eigen::Vector3d(iteratorNext->second[0],iteratorNext->second[1],iteratorNext->second[2]);
                            g2o::SE3Quat wTj = g2o::SE3Quat (localQ,localP);
                            g2o::SE3Quat iTj = wTi.inverse() * wTj;
                            RealTimeEdgeSE3Graph *e = new RealTimeEdgeSE3Graph(mLastScale);
                            e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(index)));
                            e->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(index+1)));
                            e->setMeasurement(iTj);
                            Eigen::Matrix<double, 6, 6> inforMatrix = Eigen::Matrix<double, 6, 6>::Identity();
                            inforMatrix.block<3,3>(0,0) =  0.1*inforMatrix.block<3,3>(0,0) ;
                            inforMatrix.block<3,3>(3,3) =  inforMatrix.block<3,3>(3,3) ;
                            e->setInformation(inforMatrix);
                            optimizer.addEdge(e);
                        }

                        double t = iterator->first;
                        auto iteratoriGPS = iGPSPositionMap.find(t);
                        const float thHuber = sqrt(5.991);
                        if(iteratoriGPS != iGPSPositionMap.end())
                        {
                            g2o::Vector3D iGPSPosition = g2o::Vector3D(iteratoriGPS->second.x,iteratoriGPS->second.y,iteratoriGPS->second.z);
                            RealTimeEdgeSE3iGPSFusion *e = new RealTimeEdgeSE3iGPSFusion();
                            e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(index)));
                            e->setMeasurement(iGPSPosition);
                            e->setInformation(Eigen::Matrix3d::Identity());
                            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            rk->setDelta(thHuber);
                            optimizer.addEdge(e);
                        }
                        index++;
                    }

                    optimizer.initializeOptimization();
                    optimizer.optimize(10);
                    ofstream f;
                    f.open("./RealTimeResult.txt",ios::app);
                    length = mmFusionPoseMap.size();
                    auto iter = mmFusionPoseMap.begin();

                    for(int i = 0; i< length; iter++)
                    {
                        if(length-iFixLength>0)
                        {
                            length--;
                            continue;
                        }
                        if(i < length -1 && i > length -30) // 将位姿图中的除了最后一个节点以外都设置成优化以后的pose
                        {
                            //cout << "iter->second     " << iter->second[0] << " " << iter->second[1] << " "
                            //     << iter->second[2] << endl;

                            g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(i));
                            g2o::SE3Quat SE3quat = vSE3->estimate();
                            Eigen::Quaterniond FusionQ = SE3quat.rotation();
                            Eigen::Vector3d FusionP = SE3quat.translation();
                            std::vector<double> FusionPose{FusionP.x(), FusionP.y(), FusionP.z(),
                                                           FusionQ.w(), FusionQ.x(), FusionQ.y(), FusionQ.z()};
                            iter->second = FusionPose;
                        }
                        if(i == length -1) // 将位姿图中的除了最后一个节点以外都设置成优化以后的pose
                        {
                            //cout<< "Initial estimated global localization result  = " << iter->second[0] << " "<< iter->second[1] << " " << iter->second[2]   <<endl;

                            g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(i));
                            g2o::SE3Quat SE3quat = vSE3->estimate();
                            Eigen::Quaterniond FusionQ = SE3quat.rotation();
                            Eigen::Vector3d FusionP = SE3quat.translation();
                            std::vector<double> FusionPose{ FusionP.x(),FusionP.y(),FusionP.z(),
                                                            FusionQ.w(),FusionQ.x(),FusionQ.y(),FusionQ.z()};

                            //cout<< "Estimated global localization result = " << FusionP.x() << "," << FusionP.y() << "," << FusionP.z() <<endl;
                            Eigen::Vector3d itertest = Eigen::Vector3d(FusionP.x()- iter->second[0],FusionP.y()-iter->second[1],FusionP.z()- iter->second[2]);
                            double error = itertest.norm();
                            if(error>0.1)
                            cout<< "estimated global localization error  = " << FusionP.x()- iter->second[0] << " "<< FusionP.y() - iter->second[1] << " " << FusionP.z() - iter->second[2]   <<endl;

                            iter->second = FusionPose;
                            //cout << "SE3quat         " << SE3quat.translation().transpose() <<endl;
                            f << fixed;
                            f << iter->first  << "," <<  setprecision(9) << FusionP.x() << "," << FusionP.y() << "," << FusionP.z() << "," << FusionQ.x() << "," << FusionQ.y() << "," << FusionQ.z() << ","  << FusionQ.w()  <<endl;

                            Eigen::Matrix4d WVO_T_body = Eigen::Matrix4d::Identity();
                            Eigen::Matrix4d WGPS_T_body = Eigen::Matrix4d::Identity();
                            double t = iter->first;
                            auto localPose = VOPoseMap.find(t);
                            Eigen::Quaterniond localQ(localPose->second[3],localPose->second[4],localPose->second[5],localPose->second[6]);
                            Eigen::Vector3d localP = Eigen::Vector3d(localPose->second[0],localPose->second[1],localPose->second[2]);
                            WVO_T_body.block<3,3>(0,0) = localQ.toRotationMatrix();
                            WVO_T_body.block<3,1>(0,3) = mLastScale * localP;
                            WGPS_T_body.block<3,3>(0,0) = Eigen::Quaterniond(FusionQ.w(),FusionQ.x(),FusionQ.y(),FusionQ.z()).toRotationMatrix();
                            WGPS_T_body.block<3,1>(0,3) =Eigen::Vector3d(FusionP.x(),FusionP.y(),FusionP.z());
                            Eigen::Matrix4d test = WGPS_T_body * WVO_T_body.inverse();
                            //iGPS_T_VO = WGPS_T_body * WVO_T_body.inverse();
                            iGPS_T_VO.block<3,3>(0,0) = test.block<3,3>(0,0);
                            iGPS_T_VO.block<3,1>(0,3) = test.block<3,1>(0,3);
                        }
                        i++;
                    }

                    f.close();
                    m_PoseMap.unlock();

                }
                else
                {
                    //estimate initial scale
                    std::chrono::time_point<std::chrono::system_clock> start,end;
                    start = std::chrono::system_clock::now();

                    //构造求解器
                    g2o::SparseOptimizer optimizer;
                    //使用稀疏线性方程求解器
                    g2o::BlockSolver_7_3::LinearSolverType * linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_7_3::PoseMatrixType>();
                    // 7×3的参数
                    auto * solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
                    //L-M下降算法
                    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
                    //solver->setUserLambdaInit(1e-16);

                    optimizer.setAlgorithm(solver);
                    optimizer.setVerbose(true);

                    m_PoseMap.lock();

                    //add vertex
                    auto * v = new g2o::VertexSim3Expmap();
                    v->setId(0);
                    v->setFixed(false);
                    v->setEstimate( g2o::Sim3());
                    v->setMarginalized(false);
                    optimizer.addVertex( v );

                    g2o::Vector3D VOPositionAvg(0,0,0),iGPSPositionAvg(0,0,0);
                    int iterNum = 0;
                    for(auto iterator = iGPSPositionMap.begin(); iterator != iGPSPositionMap.end();iterator++)
                    {
                        double t = iterator->first;
                        auto iterVO = mmFusionPoseMap.find(t);
                        g2o::Vector3D VOpos3d(iterVO->second[0],iterVO->second[1],iterVO->second[2]);
                        g2o::Vector3D iGPSpos3d(iterator->second.x,iterator->second.y,iterator->second.z);
                        VOPositionAvg += VOpos3d;
                        iGPSPositionAvg += iGPSpos3d;
                        iterNum++;
                    }
                    VOPositionAvg /=iterNum; iGPSPositionAvg/=iterNum;
                    double dis1 = 0,dis2 = 0;
                    iterNum = 0;
                    for(auto iterator = iGPSPositionMap.begin(); iterator != iGPSPositionMap.end();iterator++)
                    {
                        double t = iterator->first;
                        auto iterVO = mmFusionPoseMap.find(t);
                        g2o::Vector3D VOpos3d(iterVO->second[0],iterVO->second[1],iterVO->second[2]);
                        g2o::Vector3D iGPSpos3d(iterator->second.x,iterator->second.y,iterator->second.z);
                        if(iterNum == 0)
                        {
                            dis1 = (VOpos3d - VOPositionAvg).norm();
                            dis2 = (iGPSpos3d - iGPSPositionAvg).norm();
                        }
                        else
                        {
                            if((VOpos3d - VOPositionAvg).norm() <dis1)
                                dis1 = (VOpos3d - VOPositionAvg).norm();
                            if((iGPSpos3d - iGPSPositionAvg).norm() <dis2)
                                dis2 = (iGPSpos3d - iGPSPositionAvg).norm();
                        }
                        iterNum++;
                    }
                    double scale = dis2 /dis1;   //initial scale estimate
                    std::cout<< "scale = " << scale <<endl;


                    const float thHuber = sqrt(14.07);
                    for(auto iterator = iGPSPositionMap.begin(); iterator != iGPSPositionMap.end();iterator++)
                    {
                        double t = iterator->first;
                        auto iterVO = mmFusionPoseMap.find(t);
                        g2o::Vector3D VOpos3d(scale*iterVO->second[0],scale*iterVO->second[1],scale*iterVO->second[2]);
                        g2o::Vector3D iGPSpos3d(iterator->second.x,iterator->second.y,iterator->second.z);
                        //std::cout<< "VOpos3d = " << VOpos3d <<std::endl;
                        //std::cout<< "iGPSpos3d = " << iGPSpos3d <<std::endl;
                        RealTimeEdgeSim3 *e = new RealTimeEdgeSim3(VOpos3d);
                        e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                        e->setMeasurement(iGPSpos3d);
                        e->setInformation(Eigen::Matrix3d::Identity());
                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuber);
                        optimizer.addEdge(e);
                    }

                    optimizer.initializeOptimization(0);
                    optimizer.optimize(10);


                    mScale = scale * v->estimate().scale();
                    if(mScale > scale/3 && mScale < scale*3)
                    {
                        std::cout << "mScale = " << mScale << std::endl;
                        mbScaleFlag = true;
                        iGPS_T_VO.block<3,3>(0,0) = v->estimate().rotation().toRotationMatrix();
                        iGPS_T_VO.block<3,1>(0,3) = v->estimate().translation();
                    }
                    else
                    {
                        mScale = 1;
                    }

                    end = std::chrono::system_clock::now();
                    double seconds = std::chrono::duration_cast<std::chrono::duration<double> >(end - start).count();
                    //std::cout<< " seconds =  " << seconds <<std::endl;
                    m_PoseMap.unlock();
                }
            }

            usleep(5000);
            if(mpLoopCloser->isFinished())
            {
                SetFinish();
                break;
            }

        }

    }
}
