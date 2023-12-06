/*
* Modification: SQ-SLAM
* Version: 1.0
* Created: 05/15/2022
* Author: Xiao Han
*/

#ifndef G2OOBJECT_H
#define G2OOBJECT_H

#include "SuperQuadric.h"
#include <iostream>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "ObjectMap.h"

using namespace std;
using namespace ORB_SLAM2;

typedef Eigen::Matrix<double, 1, 1> Vector1d;
typedef Eigen::Matrix<double, 5, 1> Vector5d;

namespace g2o
{
class VertexSQshape : public BaseVertex<2,SuperQuadric>  
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexSQshape(){}

    virtual bool read(std::istream& is) {}
    virtual bool write(std::ostream& os) const {}

    virtual void setToOriginImpl() { _estimate = SuperQuadric(); }

    virtual void oplusImpl(const double* update_)
    {
        Eigen::Map<const Vector2d> update(update_);
        setEstimate(_estimate.ShapeUpdate(update));
    }

};

class EdgeSQshapeXYZ : public BaseUnaryEdge<1,double,VertexSQshape>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSQshapeXYZ(){}

    virtual bool read(std::istream& is) {}
    virtual bool write(std::ostream& os) const {}

    void computeError()  {
        const VertexSQshape* v1 = static_cast<const VertexSQshape*>(_vertices[0]);

        _error(0,0) = v1->estimate().ComputeError(MapPoint) + 0.0001 / N * (abs(v1->estimate().ep1 - 0.1) + abs(v1->estimate().ep2 - 0.1)) ;
    }

    Vector3d MapPoint;
    int N;

};   

/* class VertexSQshape : public BaseVertex<5,SuperQuadric>  
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexSQshape(){}

    virtual bool read(std::istream& is) {}
    virtual bool write(std::ostream& os) const {}

    virtual void setToOriginImpl() { _estimate = SuperQuadric(); }

    virtual void oplusImpl(const double* update_)
    {
        Eigen::Map<const Vector5d> update(update_);
        setEstimate(_estimate.ShapeUpdate(update));
    }

};

class EdgeSQshapeXYZ : public BaseUnaryEdge<1,double,VertexSQshape>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSQshapeXYZ(){}

    virtual bool read(std::istream& is) {}
    virtual bool write(std::ostream& os) const {}

    void computeError()  {
        const VertexSQshape* v1 = static_cast<const VertexSQshape*>(_vertices[0]);

        //_error(0,0) = v1->estimate().CumputerError(MapPoint) * v1->estimate().a1 * v1->estimate().a2 * v1->estimate().a3 ;
        //_error(0,0) = _error(0,0) + 0.01 / N * (abs(v1->estimate().ep1 - 1) + abs(v1->estimate().ep2 - 1));
        _error(0,0) = v1->estimate().ComputeError(MapPoint);
        //_error(0,0) = v1->estimate().ComputeError(MapPoint)  + 0.01 / N * (abs(v1->estimate().ep1 - 1) + abs(v1->estimate().ep2 - 1)) ;
    }

    Vector3d MapPoint;
    int N;

};  */



    class VertexSQyaw : public BaseVertex<1,Vector1d>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        VertexSQyaw(){}

        virtual bool read(std::istream& is) {}
        virtual bool write(std::ostream& os) const {}

        virtual void setToOriginImpl() { _estimate = Vector1d(0);}

        virtual void oplusImpl(const double* update_)
        {
            Eigen::Map<const Vector1d> update(update_);
            //cout<<"update: "<<update<<endl;
            setEstimate(_estimate + update);
        }
    };

    class EdgeSQRotationLine : public BaseUnaryEdge<1,double,VertexSQyaw>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeSQRotationLine(){}

        virtual bool read(std::istream& is) {}
        virtual bool write(std::ostream& os) const {}

        void computeError()  {
            const VertexSQyaw* v1 = static_cast<const VertexSQyaw*>(_vertices[0]);
            double yaw = v1->estimate()(0);
            _error(0,0) = CalculateError(yaw);
        }

        double CalculateError(double yaw);

        Eigen::Vector3d axisPointPos;
        Eigen::Vector3d twobj;
        double lineAngle;
        Eigen::Vector2d centerUV;
        SE3Quat Tcw;
        double fx,fy,cx,cy;

    };

}

namespace ORB_SLAM2
{
class Frame;
class ObjectOptimizer
{
public:
    void static OptimzeShape(SuperQuadric& SQ, const vector<Eigen::Vector3d> &points,const vector<double>& pointsCoefficient);
    float static OptimizeRotation(const Object_Map& obj,const vector<vector<int>>& linesIdx,float IniYaw,cv::Mat twobj,const Frame& CurrentFrame);

};

}



#endif