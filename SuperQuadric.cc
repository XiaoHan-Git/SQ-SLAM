/*
* Modification: SQ-SLAM
* Version: 1.0
* Created: 05/06/2022
* Author: Xiao Han
*/

#include "SuperQuadric.h"


namespace ORB_SLAM2
{

SuperQuadric::SuperQuadric()
{
}

/* SuperQuadric::SuperQuadric(const SuperQuadric& a)
{
    a1 = a.a1;
    a2 = a.a2;
    a3 = a.a3;
    ep1 = a.ep1;
    ep2 = a.ep2;
    mTobjw = a.mTobjw;
    mfMaxDist = a.mfMaxDist;
    
} */

SuperQuadric SuperQuadric::ShapeUpdate(const Eigen::Vector2d &update)
{
    SuperQuadric res(*this);
    res.ep1 = res.ep1 + update[0];
    res.ep2 = res.ep2 + update[1];
    return res;
} 

/* SuperQuadric SuperQuadric::ShapeUpdate(const Vector5d &update)
{
    SuperQuadric res(*this);
    res.ep1 = res.ep1 + update[0];
    res.ep2 = res.ep2 + update[1];
    res.a1 = res.a1 + update[2];
    res.a2 = res.a2 + update[3];
    res.a3 = res.a3 + update[4];
    
    return res;
} */


double SuperQuadric::ComputeError(const vector<Eigen::Vector3d> &points) const
{
    double SumError=0,Error=0;
    double F;
    for(size_t i=0;i<points.size();i++)
    {
        F = pow(pow(points[i].x()/a1,2),1.0/ep2) + pow(pow(points[i].y()/a2,2),1.0/ep2);
        F = pow(F,ep2/ep1) + pow(pow(points[i].z()/a3,2),1.0/ep1);
        F = pow(F,-ep1/2);
        Error = pow(points[i].norm() * (1 - F),2);
        //Error = abs(points[i].norm() * (1 - F));
        SumError = SumError + Error;
    }
    return SumError;

    /* double SumError=0,Error=0;
    double F;

    for(size_t i=0;i<points.size();i++)
    {
        F = pow(pow(points[i].x()/a1,2),1.0/ep2) + pow(pow(points[i].y()/a2,2),1.0/ep2);
        F = pow(F,ep2/ep1) + pow(pow(points[i].z()/a3,2),1.0/ep1);
        F = pow(F,ep1) - 1;
        Error = F * F;
        //Error = abs(points[i].norm() * (1 - F));
        SumError = SumError + Error;
    }
    SumError = SumError * a1 * a2 * a3;
    return SumError; */

}

double SuperQuadric::ComputeError(Eigen::Vector3d &point) const
{
    double Error=0;
    double F = pow(pow(point.x()/a1,2),1.0/ep2) + pow(pow(point.y()/a2,2),1.0/ep2);
    F = pow(F,ep2/ep1) + pow(pow(point.z()/a3,2),1.0/ep1);
    F = pow(F,-ep1/2);
    Error = pow(point.norm() * (1 - F),2);
    //Error = abs(points[i].norm() * (1 - F));
    return Error;

    /* double Error=0;
    double F = pow(pow(point.x()/a1,2),1.0/ep2) + pow(pow(point.y()/a2,2),1.0/ep2);
    F = pow(F,ep2/ep1) + pow(pow(point.z()/a3,2),1.0/ep1);
    F = pow(F,ep1) - 1;
    Error = F * F;
    //Error = abs(points[i].norm() * (1 - F));
    return Error; */
}

bool SuperQuadric::PointIn(Eigen::Vector3d &point)
{
    double F = pow(pow(point.x()/a1,2),1.0/ep2) + pow(pow(point.y()/a2,2),1.0/ep2);
    F = pow(F,ep2/ep1) + pow(pow(point.z()/a3,2),1.0/ep1) - 1;
    //cout <<"F: "<< F<< endl;
    if(F<=0)
        return true;
    else
        return false;
}

/* void SuperQuadric::SampleShape(vector<Eigen::Vector3d> &points, vector<double>& pointsCoefficient)
{   

    float res_ep1 = 0.1,res_ep2 = 0.1;
    float Error = 0,min_error = 1e5;
    float dist_min,dist_max;
    for(float sample_ep1 = 0.1;sample_ep1<2;)
    {
        for(float sample_ep2 = 0.1;sample_ep2<2;)
        {
            ep1 = sample_ep1;
            ep2 = sample_ep2;
            float maxDist = 0;
            float minDist = 1e5;
            Error = ComputeError(points,maxDist,minDist);
            //cout<<"ep1: "<<sample_ep1<<" ep2: "<<sample_ep2<<" Error: "<<Error<<endl;
            if(Error < min_error)
            {
                min_error = Error;
                res_ep1 = sample_ep1;
                res_ep2 = sample_ep2;
                dist_min = minDist;
                dist_max = maxDist;
            }
            sample_ep2+=0.2;
        }
        sample_ep1+=0.2;
    }

    ep1 = res_ep1;
    ep2 = res_ep2;
    float dist = dist_max - dist_min;
    for(int i=0,iend=points.size();i<iend;i++)
    {
        pointsCoefficient[i] = 1 - ((ComputeError(points[i]) - dist_min) / dist);
        pointsCoefficient[i] = pointsCoefficient[i] * pointsCoefficient[i];
    }

} */

void SuperQuadric::SampleShape(vector<Eigen::Vector3d> &points)
{   

    float res_ep1 = 0.1,res_ep2 = 0.1;
    float Error = 0,min_error = 1e5;
    for(float sample_ep1 = 0.1;sample_ep1<2;)
    {
        for(float sample_ep2 = 0.1;sample_ep2<2;)
        {
            ep1 = sample_ep1;
            ep2 = sample_ep2;
           
            Error = ComputeError(points);
            //cout<<"ep1: "<<sample_ep1<<" ep2: "<<sample_ep2<<" Error: "<<Error<<endl;
            if(Error < min_error)
            {
                min_error = Error;
                res_ep1 = sample_ep1;
                res_ep2 = sample_ep2;
            }
            sample_ep2+=0.2;
        }
        sample_ep1+=0.2;
    }
    ep1 = res_ep1;
    ep2 = res_ep2;
}

}
