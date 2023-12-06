/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* Modification: SQ-SLAM
* Version: 1.0
* Created: 05/20/2022
* Author: Xiao Han
*/

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <Converter.h>
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM2
{


MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }

    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    std::sort(vpKFs.begin(),vpKFs.end(),[](KeyFrame* pKF1, KeyFrame* pKF2){return pKF1->mnId<pKF2->mnId;});
    
    if(bDrawKF)
    {
        cv::Mat last_twc;
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            cv::Mat ttwc = Twc.colRange(0,3).row(3);
            if(i==0)
                last_twc = ttwc;
            else
            {   
                glLineWidth(mKeyFrameLineWidth);
                glColor3f(0.0f,1.0f,0.0f);
                glBegin(GL_LINES);
                glVertex3f(last_twc.at<float>(0),last_twc.at<float>(1),last_twc.at<float>(2));
                glVertex3f(ttwc.at<float>(0),ttwc.at<float>(1),ttwc.at<float>(2));
                last_twc = ttwc;
                glEnd();
            }

            glPushMatrix();
            glMultMatrixf(Twc.ptr<GLfloat>(0));
            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }




    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

//SQ-SLAM

void MapDrawer::DrawObject(int flag)
{
    vector<Object_Map*> AllObjs = mpMap->GetAllObjectMap();

    if(AllObjs.empty())
        return;
    
    const float w = mKeyFrameSize;

    const float linewidth = mKeyFrameLineWidth;

    // color.
    vector<vector<GLint>> colors = { {135,0,248},
                                     {255,0,253},
                                     {4,254,119},
                                     {255,126,1},
                                     {0,112,255},
                                     {0,250,250}};

    for(int i=0;i<AllObjs.size();i++)
    {
        if(AllObjs[i]->IsBad() || AllObjs[i]->mbFirstInit || AllObjs[i]->mnObs <5)
            continue;
        
        glPushMatrix();
    
        SuperQuadric SQ = AllObjs[i]->mSQ;
        Eigen::Matrix4d pose =  AllObjs[i]->mSQ.mTobjw.inverse().to_homogeneous_matrix();
        glMultMatrixd(pose.data());
        vector<vector<Eigen::Vector3f>> uvPoint;
        CreatSQPoint(SQ, uvPoint, 20);

        vector<GLint> color = colors[AllObjs[i]->mnClass % 6];
        //vector<GLint> color = {255,164,96};

        //float error = SQ->CumputerError(uvPoint[0]);
        //cout << error<<endl;
        //XYZ Coordinate
        glLineWidth(linewidth);
        glBegin ( GL_LINES );
        glColor3f ( 1.0f,0.f,0.f );
        glVertex3f( 0,0,0 );
        glVertex3f( w,0,0 );
        glColor3f( 0.f,1.0f,0.f);
        glVertex3f( 0,0,0 );
        glVertex3f( 0,w,0 );
        glColor3f( 0.f,0.f,1.f);
        glVertex3f( 0,0,0 );
        glVertex3f( 0,0,w );
        glEnd();

        //plot Quadric
        glLineWidth(linewidth);
        glBegin(GL_LINES);
        //glBegin ( GL_QUADS );
        glColor3f(color[0]/255.0, color[1]/255.0, color[2]/255.0);
        Eigen::Vector3f point;
        for (int u_idx = 0; u_idx < uvPoint.size() - 1; u_idx++)
            for (int v_idx = 0; v_idx < uvPoint[0].size() - 1; v_idx++) 
            {
                //vertex 1
                point = uvPoint[u_idx][v_idx];
                glVertex3f(point[0], point[1], point[2]);
                //vertex 2
                point = uvPoint[u_idx][v_idx + 1];
                glVertex3f(point[0], point[1], point[2]);
                //vertex 2
                point = uvPoint[u_idx][v_idx + 1];
                glVertex3f(point[0], point[1], point[2]);
                //vertex 3
                point = uvPoint[u_idx + 1][v_idx + 1];
                glVertex3f(point[0], point[1], point[2]);
                //vertex 3
                point = uvPoint[u_idx + 1][v_idx + 1];
                glVertex3f(point[0], point[1], point[2]);
                //vertex 4
                point = uvPoint[u_idx][v_idx + 1];
                glVertex3f(point[0], point[1], point[2]);
                //vertex 4
                point = uvPoint[u_idx][v_idx + 1];
                glVertex3f(point[0], point[1], point[2]);
                //vertex 1
                point = uvPoint[u_idx][v_idx];
                glVertex3f(point[0], point[1], point[2]);
            }
        glEnd();

        //Ablation Study
        // GT bbox
        /* 
        float gt_a1 = AllObjs[i]->GT_a1;
        float gt_a2 = AllObjs[i]->GT_a2;
        float gt_a3 = AllObjs[i]->GT_a3;
        g2o::SE3Quat Tow = AllObjs[i]->mSQ.mTobjw;

        glLineWidth(linewidth);
        glBegin ( GL_LINES );
        glColor3f ( 0.0f,0.f,0.f );
        glVertex3f(gt_a1,gt_a2,gt_a3);
        glVertex3f(-gt_a1,gt_a2,gt_a3);
        glVertex3f(-gt_a1,gt_a2,gt_a3);
        glVertex3f(-gt_a1,-gt_a2,gt_a3);
        glVertex3f(-gt_a1,-gt_a2,gt_a3);
        glVertex3f(gt_a1,-gt_a2,gt_a3);
        glVertex3f(gt_a1,-gt_a2,gt_a3);
        glVertex3f(gt_a1,gt_a2,gt_a3);

        glVertex3f(gt_a1,gt_a2,-gt_a3);
        glVertex3f(-gt_a1,gt_a2,-gt_a3);
        glVertex3f(-gt_a1,gt_a2,-gt_a3);
        glVertex3f(-gt_a1,-gt_a2,-gt_a3);
        glVertex3f(-gt_a1,-gt_a2,-gt_a3);
        glVertex3f(gt_a1,-gt_a2,-gt_a3);
        glVertex3f(gt_a1,-gt_a2,-gt_a3);
        glVertex3f(gt_a1,gt_a2,-gt_a3);

        glVertex3f(gt_a1,gt_a2,gt_a3);
        glVertex3f(gt_a1,gt_a2,-gt_a3);
        glVertex3f(-gt_a1,gt_a2,gt_a3);
        glVertex3f(-gt_a1,gt_a2,-gt_a3);
        glVertex3f(-gt_a1,-gt_a2,gt_a3);
        glVertex3f(-gt_a1,-gt_a2,-gt_a3);
        glVertex3f(gt_a1,-gt_a2,gt_a3);
        glVertex3f(gt_a1,-gt_a2,-gt_a3);
        glEnd();
        */

        glPopMatrix();

        if(flag == 1)
        {
            glPointSize(mPointSize);
            glBegin(GL_POINTS);
            glColor3f(color[0]/255.0, color[1]/255.0, color[2]/255.0);
            //float color1 = 160;
            //float color2 = 32;
            //float color3 = 240;
            //glColor3f(color1/255.0f,color2/255.0f,color3/255.0f);
            vector<MapPoint*> pMPs = AllObjs[i]->mvpMapPoints;

            //Ablation Study
            /* int numGT = 0;
            int numout = 0; */

            for(size_t i=0, iend=pMPs.size(); i<iend;i++)
            {
                //if(pMPs[i]->isBad())
                //    continue;
                cv::Mat pos = pMPs[i]->GetWorldPos();
                glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

                //Ablation Study
                /* Eigen::Vector3d posIno = Tow.rotation() * Converter::toVector3d(pos) + Tow.translation() ;
                if(abs(posIno[0]) < gt_a1 && abs(posIno[1]) < gt_a2 && abs(posIno[2]) < gt_a3)
                    numGT++;
                else
                    numout++; */
                
            }

            glEnd();

            //Ablation Study
            /* if(numGT >10)
            {
                float val = float(numout)/float(numGT);
                cout <<"mnId: "<<AllObjs[i]->mnId<< " val: "<<val<<endl;
                AllObjs[i]->OutMappointsNum.push_back(val);
            } */   
        
        }
        
    }

}

inline float MapDrawer::sgn(float num)
{
    if(num<0)
        return -1;
    else if (num > 0)
        return 1;
    else
        return 0;
}

inline float MapDrawer::PowOfNeg(float num,float n)
{
    return sgn(num) * pow(abs(num),n);
}


void MapDrawer::CreatSQPoint(const SuperQuadric& SQ, vector<vector<Eigen::Vector3f>>& uvPoint,int seg)
{
    float du = M_PI / seg;
    float dv = 2 * M_PI / seg;
    float u_init = -M_PI_2;
    float v_init = -M_PI;
    float u=0,v=0;
    vector<Eigen::Vector3f> col_points;
    for (int u_idx = 0; u_idx <= seg; u_idx++) {
        u = u_init + u_idx*du;
        if(u > M_PI_2)
            u = M_PI_2;
        col_points.clear();
        for (int v_idx = 0; v_idx <= seg; v_idx++) {
            v = v_init + v_idx*dv;
            if(v > M_PI)
                v = M_PI;

            float x = (SQ.a1)*PowOfNeg(cos(u),SQ.ep1) * PowOfNeg(cos(v),SQ.ep2);
            float y = (SQ.a2)*PowOfNeg(cos(u),SQ.ep1) * PowOfNeg(sin(v),SQ.ep2);
            float z = (SQ.a3)*PowOfNeg(sin(u),SQ.ep1);

            Eigen::Vector3f point(x,y,z);
            col_points.push_back(point);
        }
        uvPoint.push_back(col_points);
    }
}


} //namespace ORB_SLAM
