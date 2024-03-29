/*
* Modification: SQ-SLAM
* Version: 1.0
* Created: 05/15/2022
* Author: Xiao Han
*/

#include "OptimizeObject.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Frame.h"
#include "ObjectMap.h"
#include "Converter.h"




namespace ORB_SLAM2
{

void ObjectOptimizer::OptimzeShape(SuperQuadric& SQ, const vector<Eigen::Vector3d> &points,const vector<double>& pointsCoefficient)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverTraits<2,3>::LinearSolverType *linearSolver;
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverTraits<2,3>::PoseMatrixType>();
    g2o::BlockSolver<g2o::BlockSolverTraits<2,3>> *solver_ptr = new g2o::BlockSolver<g2o::BlockSolverTraits<2,3>> (linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    g2o::VertexSQshape *vSQShape = new g2o::VertexSQshape();
    vSQShape->setEstimate(SQ);
    vSQShape->setId(0);
    vSQShape->setFixed(false);
    optimizer.addVertex(vSQShape);

    const int N = points.size();
    Eigen::Vector3d point;
    for(int i=0;i<N;i++)
    {
        point = points[i];
        g2o::EdgeSQshapeXYZ *e = new g2o::EdgeSQshapeXYZ();
        e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(vSQShape));
        e->setInformation(Vector1d(pointsCoefficient[i]));
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk); 
        e->MapPoint[0] = point[0];
        e->MapPoint[1] = point[1];
        e->MapPoint[2] = point[2];
        e->N = N;
        optimizer.addEdge(e);

    }
    //cout<<"Before ep1: "<<vSQShape->estimate().ep1<<" ep2: "<<vSQShape->estimate().ep2<<endl;
    optimizer.initializeOptimization();
    optimizer.optimize(5);
    SQ = vSQShape->estimate();
    if(SQ.ep1 < 0.1)   SQ.ep1 = 0.1;
    else if(SQ.ep1 > 2) SQ.ep1 = 2;

    if(SQ.ep2 < 0.1)   SQ.ep2 = 0.1;
    else if(SQ.ep2 > 2)   SQ.ep2 = 2;

    //cout<<"After ep1: "<<vSQShape->estimate().ep1<<" ep2: "<<vSQShape->estimate().ep2<<endl;

}  


/* void ObjectOptimizer::OptimzeShape(SuperQuadric& SQ, const vector<Eigen::Vector3d> &points, const vector<double>& pointsCoefficient)
{

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverTraits<5,3>::LinearSolverType *linearSolver;
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverTraits<5,3>::PoseMatrixType>();
    g2o::BlockSolver<g2o::BlockSolverTraits<5,3>> *solver_ptr = new g2o::BlockSolver<g2o::BlockSolverTraits<5,3>> (linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    g2o::VertexSQshape *vSQShape = new g2o::VertexSQshape();
    vSQShape->setEstimate(SQ);
    vSQShape->setId(0);
    vSQShape->setFixed(false);
    optimizer.addVertex(vSQShape);

    const int N = points.size();
    Eigen::Vector3d point;
    for(int i=0;i<N;i++)
    {
        point = points[i];
        g2o::EdgeSQshapeXYZ *e = new g2o::EdgeSQshapeXYZ();
        e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(vSQShape));
        e->setInformation(Vector1d(pointsCoefficient[i]));
        //g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        //e->setRobustKernel(rk);
        e->MapPoint[0] = point[0];
        e->MapPoint[1] = point[1];
        e->MapPoint[2] = point[2];
        e->N = N;
        optimizer.addEdge(e);

    }
    //cout<<"Before ep1: "<<vSQShape->estimate().ep1<<" ep2: "<<vSQShape->estimate().ep2<<endl;
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    
    SuperQuadric resSQ = vSQShape->estimate();
    
    if(abs(SQ.a1 - resSQ.a1) < SQ.a1 / 2 && abs(SQ.a2 - resSQ.a2) < SQ.a2 / 2 && abs(SQ.a3 - resSQ.a3) < SQ.a3 / 2)
    {
        SQ.a1 = resSQ.a1;
        SQ.a2 = resSQ.a2;
        SQ.a3 = resSQ.a3;
    }
        

    SQ.ep1 = resSQ.ep1;
    SQ.ep2 = resSQ.ep2;
    SQ.mTobjw = resSQ.mTobjw;
    if(SQ.ep1 < 0.1)   SQ.ep1 = 0.1;
    else if(SQ.ep1 > 2) SQ.ep1 = 2;

    if(SQ.ep2 < 0.1)   SQ.ep2 = 0.1;
    else if(SQ.ep2 > 2)   SQ.ep2 = 2;
    //cout<<"After ep1: "<<vSQShape->estimate().ep1<<" ep2: "<<vSQShape->estimate().ep2<<endl;

}  */


float ObjectOptimizer::OptimizeRotation(const Object_Map& obj, const vector<vector<int>>& linesIdx,float IniYaw,cv::Mat twobj,const Frame& CurrentFrame)
{

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverTraits<1,1>::LinearSolverType *linearSolver;
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverTraits<1,1>::PoseMatrixType>();
    g2o::BlockSolver<g2o::BlockSolverTraits<1,1>> *solver_ptr = new g2o::BlockSolver<g2o::BlockSolverTraits<1,1>> (linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    g2o::VertexSQyaw *vSQyaw = new g2o::VertexSQyaw();
    vSQyaw->setEstimate(Vector1d(IniYaw));
    vSQyaw->setId(0);
    vSQyaw->setFixed(false);
    optimizer.addVertex(vSQyaw);

    Eigen::Vector3d twobj_eigen = Converter::toVector3d(twobj);
    Eigen::Vector2d centerUV;
    cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = CurrentFrame.mTcw.rowRange(0,3).col(3);
    cv::Mat centerFramePos =  Rcw * twobj + tcw;
    float inv_z = 1.0 / centerFramePos.at<float>(2);
    centerUV(0) = CurrentFrame.fx *  centerFramePos.at<float>(0) * inv_z + CurrentFrame.cx;
    centerUV(1) = CurrentFrame.fy *  centerFramePos.at<float>(1) * inv_z + CurrentFrame.cy;

    for(int i=0;i<linesIdx.size();i++)
    {
        for(int j=0;j<linesIdx[i].size();j++)
        {

            g2o::EdgeSQRotationLine *e = new g2o::EdgeSQRotationLine();
            e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(vSQyaw));
            e->setInformation(Vector1d(1));
            //g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            //e->setRobustKernel(rk);
            Eigen::Vector3d axisPointPos = Eigen::Vector3d::Zero();
            axisPointPos(i) = obj.mfLength;
            e->axisPointPos = axisPointPos;
            e->twobj = twobj_eigen;
            Eigen::Vector4d line = obj.mlatestFrameLines.row(linesIdx[i][j]);
            e->lineAngle = atan2(line(3) - line(1), line(2) - line(0));
            e->centerUV = centerUV;
            e->Tcw = SE3Quat(Converter::toSE3Quat(CurrentFrame.mTcw));
            e->fx = CurrentFrame.fx;
            e->fy = CurrentFrame.fy;
            e->cx = CurrentFrame.cx;
            e->cy = CurrentFrame.cy;

            optimizer.addEdge(e);
        }
    }

    //cout<<"Before yaw: "<<vSQyaw->estimate()<<endl;
    optimizer.initializeOptimization();
    optimizer.optimize(5);
    //cout<<"After yaw: "<<vSQyaw->estimate()<<endl;
    return vSQyaw->estimate()(0);

}

}

namespace g2o
{


double EdgeSQRotationLine::CalculateError(double yaw)
{
    //Eigen::Matrix3d Rwobj = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Rwobj = Converter::eulerAnglesToMatrix(yaw);

    Eigen::Vector3d axisPos =  Tcw * (Rwobj * axisPointPos + twobj);
    double inv_z = 1.0 / axisPos(2);
    double u = fx * axisPos(0) * inv_z + cx;
    double v = fy * axisPos(1) * inv_z + cy;

    double angle;
    if( u < centerUV(0))
        angle = atan2(centerUV(1) - v, centerUV(0) - u);
    else
        angle = atan2(v - centerUV(1), u - centerUV(0));

    //cout<<"angle: "<<angle<<endl;

    double angle_error = abs((angle - lineAngle) * 180.0 / M_PI);
    //cout<<"angle_error: "<<angle_error<<endl;
    return min(angle_error ,180 - angle_error);
    
}
}