/*
* Modification: SQ-SLAM
* Version: 1.0
* Created: 05/6/2022
* Author: Xiao Han
*/

#ifndef SUPERQUADRIC_H
#define SUPERQUADRIC_H
#include <iostream>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include <mutex>
using namespace std;
using namespace g2o;

typedef Eigen::Matrix<double, 5, 1> Vector5d;

namespace ORB_SLAM2
{
class SuperQuadric
{
public:
    SuperQuadric();

    //SuperQuadric(const SuperQuadric& a);
    
    SuperQuadric ShapeUpdate(const Eigen::Vector2d &update);
    
    //SuperQuadric ShapeUpdate(const Vector5d &update);

    double ComputeError(const vector<Eigen::Vector3d> &points) const;
  
    double ComputeError(Eigen::Vector3d &point) const;

    bool PointIn(Eigen::Vector3d &point);
    
    void SampleShape(vector<Eigen::Vector3d> &points, vector<double>& pointsCoefficient);

    void SampleShape(vector<Eigen::Vector3d> &points);
public:

    // 11 DOF
    // pose
    SE3Quat mTobjw;
    // size
    double a1,a2,a3;
    float mfMaxDist;
    // shape
    double ep1 = 1;
    double ep2 = 1;

};

}




#endif