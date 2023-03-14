/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ViewAR.h"

#include <opencv2/highgui/highgui.hpp>

#include <mutex>
#include <thread>
#include <cstdlib>

#include <future>
#include <queue>

#include <pangolin/pangolin.h>
#include <pangolin/geometry/geometry.h>
#include <pangolin/gl/glsl.h>
#include <pangolin/gl/glvbo.h>

#include <pangolin/utils/file_utils.h>

#include <pangolin/geometry/geometry_ply.h>
#include <pangolin/geometry/glgeometry.h>

#include "Shader.h"
#include "RenderTree.h"
#include "Util.h"

#include <Eigen/SVD>
#include <Eigen/Geometry>

using namespace std;

namespace ORB_SLAM3
{

const float eps = 1e-4;

cv::Mat ExpSO3(const float &x, const float &y, const float &z)
{
    cv::Mat I = cv::Mat::eye(3,3,CV_32F);
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);
    cv::Mat W = (cv::Mat_<float>(3,3) << 0, -z, y,
                 z, 0, -x,
                 -y,  x, 0);
    if(d<eps)
        return (I + W + 0.5f*W*W);
    else
        return (I + W*sin(d)/d + W*W*(1.0f-cos(d))/d2);
}

cv::Mat ExpSO3(const cv::Mat &v)
{
    return ExpSO3(v.at<float>(0),v.at<float>(1),v.at<float>(2));
}

ViewerAR::ViewerAR(){}

void ViewerAR::Run()
{
    int w,h,wui;

    cv::Mat im, Tcw;
    int status;
    vector<cv::KeyPoint> vKeys;
    vector<MapPoint*> vMPs;

    while(1)
    {
        GetImagePose(im,Tcw,status,vKeys,vMPs);
        if(im.empty())
            cv::waitKey(mT);
        else
        {
            w = im.cols;
            h = im.rows;
            break;
        }
    }

    wui=200;

//创建GUI窗口
    pangolin::CreateWindowAndBind("Viewer",w+wui,h);

//开启gl深度测试，使得pangolin只会绘制朝向镜头的那一面像素点，避免容易混淆的透视关系出现
    glEnable(GL_DEPTH_TEST);
    //gl颜色混合
    glEnable (GL_BLEND);

    pangolin::CreatePanel("menuAR").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(wui));
    pangolin::Var<bool> menu_detectplane_fish("menuAR.Fish",false,false);
    pangolin::Var<bool> menu_detectplane_dinosaur("menuAR.dinosaur",false,false);
    pangolin::Var<bool> menu_detectplane_frog("menuAR.Frog",false,false);
    pangolin::Var<bool> menu_detectplane_pineapple("menuAR.PineApple",false,false);


    pangolin::Var<bool> menu_clear("menuAR.Clear All",false,false);
    pangolin::Var<bool> menu_drawim("menuAR.Draw Image",true,true);
    pangolin::Var<bool> menu_drawcube("menuAR.Draw OBJ",true,true);
    // pangolin::Var<float> menu_cubesize("menu. Cube Size",0.05,0.01,0.3);
    pangolin::Var<float> x_rotate("menuAR.X Rotate",250,0,360);
    pangolin::Var<float> y_rotate("menuAR.Y Rotate",176,0,360);
    pangolin::Var<float> z_rotate("menuAR.Z Rotate",92,0,360);
    pangolin::Var<float> menu_scale("menuAR.OBJ Scale",0.005,0.001,0.2);
    pangolin::Var<bool> menu_drawgrid("menuAR.Draw Grid",false,true);
    pangolin::Var<int> menu_ngrid("menuAR. Grid Elements",3,1,10);
    pangolin::Var<float> menu_sizegrid("menuAR. Element Size",0.05,0.01,0.3);
    pangolin::Var<bool> menu_drawpoints("menuAR.Draw Points",false,true);
    pangolin::Var<bool> menu_LocalizationMode("menuAR.Localization Mode",false,true);
    bool bLocalizationMode = false;

    pangolin::View& d_image = pangolin::Display("image")
            .SetBounds(0,1.0f,pangolin::Attach::Pix(wui),1.0f,(float)w/h)
            .SetLock(pangolin::LockLeft, pangolin::LockTop);

    pangolin::GlTexture imageTexture(w,h,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);
   //OpenGlMatrixSpec继承于OpenGlMatrix
    pangolin::OpenGlMatrixSpec P = pangolin::ProjectionMatrixRDF_TopLeft(w,h,fx,fy,cx,cy,0.001,1000);

    enum class RenderMode { uv=0, tex, color, normal, matcap, vertex, num_modes };
    const std::string mode_names[] = {"SHOW_UV", "SHOW_TEXTURE", "SHOW_COLOR", "SHOW_NORMAL", "SHOW_MATCAP"};
    //定义current_mod，并为其初始化
    RenderMode current_mode = RenderMode::tex;
    pangolin::AxisDirection spin_direction = pangolin::AxisNone;
    std::vector<std::future<pangolin::Geometry>> geom_to_load;
    //ExpandGlobOption函数将model命令标志后的参数提取出来
    std::vector<std::string>  fn;
    //键入与执行文件的相对位置
    //fn.push_back("./arm.obj");

    //Load Any matcap materials
    //加载材质材料
    std::vector<std::string>  mn;
    
    //266,176,254, scale=0.023
    fn.push_back("/home/spurs/dataset/3d/3d/fish/fish.obj");
    //mn.push_back("/home/spurs/dataset/3d/3d/fish/fish.jpg");

    // 27,210,199, scale=0.14
    fn.push_back("/home/spurs/dataset/3d/3d/dinosaur/dinosaur.obj");

    //250,176,92,scale=0.04
    fn.push_back("/home/spurs/dataset/3d/3d/frog/frog.obj");

    // 84,4,4, scale=0.014
    fn.push_back("/home/spurs/dataset/3d/3d/pineapple/pineapple.obj");

    
    for(const auto& filename : fn)
    {
        geom_to_load.emplace_back(std::async(std::launch::async,[filename](){
            return pangolin::LoadGeometry(filename);
        }) );
    }

    std::vector<pangolin::GlTexture> matcaps = ORBSLAM3::TryLoad<pangolin::GlTexture>(mn, [](const std::string& f){
        return pangolin::GlTexture(pangolin::LoadImage(f));
    });

    // Render tree for holding object position
    //用于保存对象位置的渲染树
    ORBSLAM3::RenderNode root;

    //std::vector<std::shared_ptr<ORBSLAM3::GlGeomRenderable>> renderables;
    auto spin_transform = std::make_shared<ORBSLAM3::SpinTransform>(spin_direction);

    // Pull one piece of loaded geometry onto the GPU if ready
    //如果准备好了，将一块加载的几何图形拉到 GPU 上
    int geom_index = 0;
    vector<Plane*> vpPlane;
    //vector<ORBSLAM3::RenderNode*> vpRoot;

    auto LoadGeometryToGpu = [&]()
    {
        // cout << "加载模型到GPU" << endl;
        cout << "geom_index=" << geom_index << ", geom_to_load.size()=" << geom_to_load.size() << endl;
        if (geom_index > geom_to_load.size() - 1)
            geom_index = 0;
        
        //for(auto& future_geom : geom_to_load) {
            if( geom_to_load[geom_index].valid() && ORBSLAM3::is_ready(geom_to_load[geom_index]) ) {
                //cout << "loading to gpu" << endl;
                auto geom = geom_to_load[geom_index].get();
                auto aabb = pangolin::GetAxisAlignedBox(geom);

                auto renderable = std::make_shared<ORBSLAM3::GlGeomRenderable>(pangolin::ToGlGeometry(geom), aabb);
                //renderables.push_back(renderable);
                ORBSLAM3::RenderNode::Edge edge = { spin_transform, { renderable, {} } };
                root.edges.clear();
                root.edges.emplace_back(std::move(edge));
                //break;
            }
        //}
    };

    
    // GlSl Graphics shader program for display
    //用于显示GlSl图形的着色器程序
    pangolin::GlSlProgram default_prog;
    auto LoadProgram = [&](const RenderMode mode){
        current_mode = mode;
        default_prog.ClearShaders();
        std::map<std::string,std::string> prog_defines;
        for(int i=0; i < (int)RenderMode::num_modes-1; ++i) {
            prog_defines[mode_names[i]] = std::to_string((int)mode == i);
        }
        default_prog.AddShader(pangolin::GlSlAnnotatedShader, pangolin::default_model_shader, prog_defines);
        default_prog.Link();
    };

    LoadProgram(current_mode);

    while(1)
    {

        if(menu_LocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menu_LocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 激活相机视图
        d_image.Activate();
        glColor3f(1.0,1.0,1.0);

        // 从 SLAM 获取最后一张图像及其计算的姿势
        GetImagePose(im,Tcw,status,vKeys,vMPs);

        // 将文本添加到图像
        PrintStatus(status,bLocalizationMode,im);

        if(menu_drawpoints)
            DrawTrackedPoints(vKeys,vMPs,im);

        // Draw image
        if(menu_drawim)
            DrawImageTexture(imageTexture,im);

        glClear(GL_DEPTH_BUFFER_BIT);

        // 切换到投影矩阵
        glMatrixMode(GL_PROJECTION);

        //加载到投影矩阵栈顶
        P.Load();

        //切换到模型视图矩阵
        glMatrixMode(GL_MODELVIEW);
        //There is a stack of matrices for each ofthe matrix modes. In GL_MODELVIEW mode, the stack depth is at least 32.In the other two modes,GL_PROJECTION and GL_TEXTURE, the depthis at least 2.The currentmatrix in any mode is the matrix on the top of the stack for that mode.
        //每个矩阵模式都有一堆矩阵。在GL_MODELVIEW模式下，堆栈深度至少为 32。在其他两种模式GL_PROJECTION和GL_TEXTURE中，深度至少为 2。任何模式下的 currentmatrix 都是该模式下堆栈顶部的矩阵。
        //任何类型矩阵初始值都为单位矩阵

        // 加载相机位姿
        //将当前矩阵(GL_MODELVIEW)替换为m
        //当前MOSELVIEW栈为Tcw
        LoadCameraPose(Tcw);
        //Tcw->null

        // 绘制虚拟物体
        if(status==2)
        {

            if(menu_clear)
            {
                if(!vpPlane.empty())
                {
                    for(size_t i=0; i<vpPlane.size(); i++)
                    {
                        delete vpPlane[i];
                    }
                    vpPlane.clear();
                    cout << "All OBJ erased!" << endl;
                }
                menu_clear = false;
            }

            bool draw_geom = false;
            float xx_rotate, yy_rotate, zz_rotate, mscale;

            if (menu_detectplane_fish)
            {
                geom_index = 0;
                draw_geom = true;
                //266,176,254, scale=0.023
                xx_rotate = 266;
                yy_rotate = 176;
                zz_rotate = 250;
                mscale = 0.013;

            }
            
            else if (menu_detectplane_dinosaur)
            {
                geom_index = 1;
                draw_geom = true;
                // 27,210,199, scale=0.14
                xx_rotate = 27.0;
                yy_rotate = 210.0;
                zz_rotate = 199.0;
                mscale = 0.4;
            }
            
            else if (menu_detectplane_frog)
            {
                geom_index = 2;
                draw_geom = true;
                //250,176,92,scale=0.04
                xx_rotate = 250;
                yy_rotate = 176;
                zz_rotate = 92;
                mscale = 0.03;
            }
            else if (menu_detectplane_pineapple)
            {
                geom_index = 3;
                draw_geom = true;
                // 84,4,4, scale=0.014
                xx_rotate = 84.0;
                yy_rotate = 4.0;
                zz_rotate = 4.0;
                mscale = 0.01;
            }

            if(draw_geom)
            {
              //迭代次数为50次
                Plane* pPlane = DetectPlane(Tcw,vMPs,50);
                if(pPlane)
                {
                    cout << "New virtual OBJ inserted!" << endl;
                    pPlane->geom_index = geom_index;
                    pPlane->x_rotate = xx_rotate;
                    pPlane->y_rotate = yy_rotate;
                    pPlane->z_rotate = zz_rotate;
                    pPlane->scale = mscale;
                    vpPlane.push_back(pPlane);
                }
                else
                {
                    cout << "No plane detected. Point the camera to a planar region." << endl;
                }
                menu_detectplane_fish = false;
                menu_detectplane_dinosaur = false;
                menu_detectplane_frog = false;
                menu_detectplane_pineapple = false;
                draw_geom = false;
            }


            if(!vpPlane.empty())
            {
                //如果存在闭环或全局 BA，则重新计算平面
                // 在本地化模式下，地图没有更新，所以我们不需要重新计算
                bool bRecompute = false;
                if(!bLocalizationMode)
                {
                    if(mpSystem->MapChanged())
                    {
                        cout << "Map changed. All virtual elements are recomputed!" << endl;
                        bRecompute = true;
                    }
                }
                
                LoadProgram(current_mode);
                for(int i=0; i<vpPlane.size(); i++)
                {
                    Plane* pPlane = vpPlane[i];

                    if(pPlane)
                    {
                        if(bRecompute)
                        {
                            pPlane->Recompute();
                        }
                        //Tcw->Tcw
                        glPushMatrix();
                        //Tcw*glTpw->Tcw
                        pPlane->glTpw.Multiply();
                        // 绘制虚拟模型
                        if(menu_drawcube)
                        {
                          glPushMatrix();
                          geom_index = pPlane->geom_index;
                          LoadGeometryToGpu();
                          // cout << "加载模型到GPU" << endl;

                          pangolin::OpenGlMatrix Trans;
                          Trans.m[0] = 1.0;
                          Trans.m[1] = 0.0;
                          Trans.m[2] = 0.0;
                          Trans.m[3] = 0.0;

                          Trans.m[4] = 0.0;
                          Trans.m[5] = -1.0;
                          Trans.m[6] = 0.0;
                          Trans.m[7] = 0.0;

                          Trans.m[8] = 0.0;
                          Trans.m[9] = 0.0;
                          Trans.m[10] = -1.0;
                          Trans.m[11] = 0.0;

                          Trans.m[12] = 0.0;
                          Trans.m[13] = 0.0;
                          Trans.m[14] = 0.0;
                          Trans.m[15] = 1.0;

                          //以Pangolin创建虚拟相机投影矩阵的函数定义投影矩阵
                          pangolin::OpenGlMatrixSpec projectionmatrix = pangolin::ProjectionMatrixRUB_BottomLeft(w,h,fx,fy,cx,cy,0.001,1000);

                          //模型绕轴旋转
                          //glRotatef(270,1,0,0);
                          //glRotatef(x_rotate,1,0,0);
                          //glRotatef(y_rotate,0,1,0);
                          //glRotatef(z_rotate,0,0,1);

                          glRotatef(pPlane->x_rotate,1,0,0);
                          glRotatef(pPlane->y_rotate,0,1,0);
                          glRotatef(pPlane->z_rotate,0,0,1);

                          //模型放缩
                          //glScalef(menu_scale,menu_scale,menu_scale);
                          glScalef(pPlane->scale, pPlane->scale, pPlane->scale);

                          //定义模型矩阵
                          float modelviewmat[16];
                          glGetFloatv(GL_MODELVIEW_MATRIX, modelviewmat);
                          pangolin::OpenGlMatrix modelviewmatrix = Trans * Mat2Matrix(modelviewmat);


                          //加载模型到视窗
                          default_prog.Bind();
                          /*
                          ORBSLAM3::render_tree(
                              default_prog, root, projectionmatrix,modelviewmatrix,
                              matcaps.size() ? &matcaps[matcap_index] : nullptr
                          );
                          */
                          
                          ORBSLAM3::render_tree(
                              default_prog, root, projectionmatrix,modelviewmatrix,
                              nullptr
                          );
                          
                          default_prog.Unbind();
                            
                          glPopMatrix();
                          // DrawCube(w,h,menu_cubesize);
                        }

                        // Draw grid plane
                        if(menu_drawgrid)
                        {
                            DrawPlane(menu_ngrid,menu_sizegrid);
                        }

                        glPopMatrix();
                    }
                }
            }


        }

        pangolin::FinishFrame();
        usleep(mT*1000);
    }

}

void ViewerAR::SetImagePose(const cv::Mat &im, const cv::Mat &Tcw, const int &status, const vector<cv::KeyPoint> &vKeys, const vector<MapPoint*> &vMPs)
{
    unique_lock<mutex> lock(mMutexPoseImage);
    mImage = im.clone();
    mTcw = Tcw.clone();
    mStatus = status;
    mvKeys = vKeys;
    mvMPs = vMPs;
}

void ViewerAR::GetImagePose(cv::Mat &im, cv::Mat &Tcw, int &status, std::vector<cv::KeyPoint> &vKeys,  std::vector<MapPoint*> &vMPs)
{
    unique_lock<mutex> lock(mMutexPoseImage);
    //image图像拷贝
    im = mImage.clone();
    //外参矩阵拷贝
    Tcw = mTcw.clone();
    status = mStatus;
    vKeys = mvKeys;
    vMPs = mvMPs;
}

pangolin::OpenGlMatrix ViewerAR::Mat2Matrix(const float mat[16])
{
  pangolin::OpenGlMatrix matrix;
  matrix.m[0] = mat[0];
  matrix.m[1] = mat[1];
  matrix.m[2] = mat[2];
  matrix.m[3]  = mat[3];

  matrix.m[4] = mat[4];
  matrix.m[5] =  mat[5];
  matrix.m[6] =  mat[6];
  matrix.m[7]  =  mat[7];

  matrix.m[8] =  mat[8];
  matrix.m[9] =  mat[9];
  matrix.m[10] =  mat[10];
  matrix.m[11]  =  mat[11];

  matrix.m[12] =  mat[12];
  matrix.m[13] =  mat[13];
  matrix.m[14] =  mat[14];
  matrix.m[15]  =  mat[15];

  return matrix;
}

void ViewerAR::LoadCameraPose(const cv::Mat &Tcw)
{
    if(!Tcw.empty())
    {
      pangolin::OpenGlMatrix M;
      //m的排列方式为正常矩阵的转置
        //Tcw = R[3,3] t[3,1]
        //            0[1,3] 1[1,1]
        //即外参矩阵T
        M.m[0] = Tcw.at<float>(0,0);
        M.m[1] = Tcw.at<float>(1,0);
        M.m[2] = Tcw.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Tcw.at<float>(0,1);
        M.m[5] = Tcw.at<float>(1,1);
        M.m[6] = Tcw.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Tcw.at<float>(0,2);
        M.m[9] = Tcw.at<float>(1,2);
        M.m[10] = Tcw.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = Tcw.at<float>(0,3);
        M.m[13] = Tcw.at<float>(1,3);
        M.m[14] = Tcw.at<float>(2,3);
        M.m[15]  = 1.0;
        //将当前矩阵设置为M
        M.Load();
        // std::cout << "Tcw参数\n" << cv::format(Tcw, cv::Formatter::FMT_DEFAULT) << std::endl;
    }
}

void ViewerAR::PrintStatus(const int &status, const bool &bLocMode, cv::Mat &im)
{
    if(!bLocMode)
    {
        switch(status)
        {
        case 1:  {AddTextToImage("SLAM NOT INITIALIZED",im,255,0,0); break;}
        case 2:  {AddTextToImage("SLAM ON",im,0,255,0); break;}
        case 3:  {AddTextToImage("SLAM LOST",im,255,0,0); break;}
        }
    }
    else
    {
        switch(status)
        {
        case 1:  {AddTextToImage("SLAM NOT INITIALIZED",im,255,0,0); break;}
        case 2:  {AddTextToImage("LOCALIZATION ON",im,0,255,0); break;}
        case 3:  {AddTextToImage("LOCALIZATION LOST",im,255,0,0); break;}
        }
    }
}

void ViewerAR::AddTextToImage(const string &s, cv::Mat &im, const int r, const int g, const int b)
{
    int l = 10;
    //imText.rowRange(im.rows-imText.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(im,s,cv::Point(l,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l-1,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l+1,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l-1,im.rows-(l-1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l,im.rows-(l-1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l+1,im.rows-(l-1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l-1,im.rows-(l+1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l,im.rows-(l+1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l+1,im.rows-(l+1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);

    cv::putText(im,s,cv::Point(l,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(r,g,b),2,8);
}

void ViewerAR::DrawImageTexture(pangolin::GlTexture &imageTexture, cv::Mat &im)
{
    if(!im.empty())
    {
        imageTexture.Upload(im.data,GL_RGB,GL_UNSIGNED_BYTE);
        imageTexture.RenderToViewportFlipY();
    }
}

//需要重构的函数
void ViewerAR::DrawCube(const float &size,const float x, const float y, const float z)
{
  pangolin::OpenGlMatrix M = pangolin::OpenGlMatrix::Translate(-x,-size-y,-z);
  glPushMatrix();
  M.Multiply();
  pangolin::glDrawColouredCube(-size,size);
  glPopMatrix();
}

void ViewerAR::DrawPlane(Plane *pPlane, int ndivs, float ndivsize)
{
    //当前栈为Tcw*glTpw->Tcw
    glPushMatrix();
    //Tcw*glTpw->Tcw*glTpw->Tcw
    pPlane->glTpw.Multiply();
    //Tcw*glTpw*glTpw->Tcw*glTpw->Tcw
    DrawPlane(ndivs,ndivsize);
    glPopMatrix();
    //Tcw*glTpw->Tcw
}

void ViewerAR::DrawPlane(int ndivs, float ndivsize)
{
    // Plane parallel to x-z at origin with normal -y
    const float minx = -ndivs*ndivsize;
    const float minz = -ndivs*ndivsize;
    const float maxx = ndivs*ndivsize;
    const float maxz = ndivs*ndivsize;


    glLineWidth(2);
    glColor3f(0.7f,0.7f,1.0f);
    glBegin(GL_LINES);

    for(int n = 0; n<=2*ndivs; n++)
    {
        glVertex3f(minx+ndivsize*n,0,minz);
        glVertex3f(minx+ndivsize*n,0,maxz);
        glVertex3f(minx,0,minz+ndivsize*n);
        glVertex3f(maxx,0,minz+ndivsize*n);
    }

    glEnd();

}

void ViewerAR::DrawTrackedPoints(const std::vector<cv::KeyPoint> &vKeys, const std::vector<MapPoint *> &vMPs, cv::Mat &im)
{
    const int N = vKeys.size();


    for(int i=0; i<N; i++)
    {
        if(vMPs[i])
        {
            cv::circle(im,vKeys[i].pt,1,cv::Scalar(0,255,0),-1);
        }
    }
}

//查找平面
Plane* ViewerAR::DetectPlane(const cv::Mat Tcw, const std::vector<MapPoint*> &vMPs, const int iterations)
{
    // Retrieve 3D points
    //取得3D点
    vector<cv::Mat> vPoints;
    vPoints.reserve(vMPs.size());
    vector<MapPoint*> vPointMP;
    vPointMP.reserve(vMPs.size());

    for(size_t i=0; i<vMPs.size(); i++)
    {
        MapPoint* pMP=vMPs[i];
        if(pMP)
        {
            if(pMP->Observations()>5)
            {
                cv::Mat Xw = (cv::Mat_<float>(3,1) << pMP->GetWorldPos()(0), pMP->GetWorldPos()(1), pMP->GetWorldPos()(2));
                vPoints.push_back(Xw);
                vPointMP.push_back(pMP);
            }
        }
    }

    const int N = vPoints.size();

    if(N<50)
        return NULL;


    // Indices for minimum set selection
    //最小集合选择的索引
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);
    vector<size_t> vAvailableIndices;

    for(int i=0; i<N; i++)
    {
        vAllIndices.push_back(i);
    }

    float bestDist = 1e10;
    vector<float> bestvDist;

    //RANSAC
    for(int n=0; n<iterations; n++)
    {
        vAvailableIndices = vAllIndices;

        cv::Mat A(3,4,CV_32F);
        A.col(3) = cv::Mat::ones(3,1,CV_32F);

        // Get min set of points
        //获取最小的点集
        for(short i = 0; i < 3; ++i)
        {
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);

            int idx = vAvailableIndices[randi];

            A.row(i).colRange(0,3) = vPoints[idx].t();

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        cv::Mat u,w,vt;
        cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

        const float a = vt.at<float>(3,0);
        const float b = vt.at<float>(3,1);
        const float c = vt.at<float>(3,2);
        const float d = vt.at<float>(3,3);

        vector<float> vDistances(N,0);

        const float f = 1.0f/sqrt(a*a+b*b+c*c+d*d);

        for(int i=0; i<N; i++)
        {
            vDistances[i] = fabs(vPoints[i].at<float>(0)*a+vPoints[i].at<float>(1)*b+vPoints[i].at<float>(2)*c+d)*f;
        }

        vector<float> vSorted = vDistances;
        sort(vSorted.begin(),vSorted.end());

        int nth = max((int)(0.2*N),20);
        const float medianDist = vSorted[nth];

        if(medianDist<bestDist)
        {
            bestDist = medianDist;
            bestvDist = vDistances;
        }
    }

    // Compute threshold inlier/outlier
    //计算阈值内点/异常值
    const float th = 1.4*bestDist;
    vector<bool> vbInliers(N,false);
    int nInliers = 0;
    for(int i=0; i<N; i++)
    {
        if(bestvDist[i]<th)
        {
            nInliers++;
            vbInliers[i]=true;
        }
    }

    vector<MapPoint*> vInlierMPs(nInliers,NULL);
    int nin = 0;
    for(int i=0; i<N; i++)
    {
        if(vbInliers[i])
        {
            vInlierMPs[nin] = vPointMP[i];
            nin++;
        }
    }

    return new Plane(vInlierMPs,Tcw);
}


Plane::Plane(const std::vector<MapPoint *> &vMPs, const cv::Mat &Tcw):mvMPs(vMPs),mTcw(Tcw.clone())
{
    rang = -3.14f/2+((float)rand()/RAND_MAX)*3.14f;
    Recompute();
}

void Plane::Recompute()
{
    const int N = mvMPs.size();

    // Recompute plane with all points
    //用所有点重新计算平面
    cv::Mat A = cv::Mat(N,4,CV_32F);
    A.col(3) = cv::Mat::ones(N,1,CV_32F);

    o = cv::Mat::zeros(3,1,CV_32F);

    int nPoints = 0;
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvMPs[i];
        if(!pMP->isBad())
        {
            cv::Mat Xw = (cv::Mat_<float>(3,1) << pMP->GetWorldPos()(0), pMP->GetWorldPos()(1), pMP->GetWorldPos()(2));
            o+=Xw;
            A.row(nPoints).colRange(0,3) = Xw.t();
            nPoints++;
        }
    }
    A.resize(nPoints);

    cv::Mat u,w,vt;
    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    float a = vt.at<float>(3,0);
    float b = vt.at<float>(3,1);
    float c = vt.at<float>(3,2);

    o = o*(1.0f/nPoints);
    const float f = 1.0f/sqrt(a*a+b*b+c*c);

    // Compute XC just the first time
    //第一次计算XC
    if(XC.empty())
    {
        cv::Mat Oc = -mTcw.colRange(0,3).rowRange(0,3).t()*mTcw.rowRange(0,3).col(3);
        XC = Oc-o;
    }

    if((XC.at<float>(0)*a+XC.at<float>(1)*b+XC.at<float>(2)*c)>0)
    {
        a=-a;
        b=-b;
        c=-c;
    }

    const float nx = a*f;
    const float ny = b*f;
    const float nz = c*f;

    n = (cv::Mat_<float>(3,1)<<nx,ny,nz);

    cv::Mat up = (cv::Mat_<float>(3,1) << 0.0f, 1.0f, 0.0f);

    cv::Mat v = up.cross(n);
    const float sa = cv::norm(v);
    const float ca = up.dot(n);
    const float ang = atan2(sa,ca);
    Tpw = cv::Mat::eye(4,4,CV_32F);
    //Tpw为4*4的对角矩阵

    Tpw.rowRange(0,3).colRange(0,3) = ExpSO3(v*ang/sa)*ExpSO3(up*rang);
    o.copyTo(Tpw.col(3).rowRange(0,3));
    //m的排列方式为正常矩阵的转置
    glTpw.m[0] = Tpw.at<float>(0,0);
    glTpw.m[1] = Tpw.at<float>(1,0);
    glTpw.m[2] = Tpw.at<float>(2,0);
    glTpw.m[3]  = 0.0;

    glTpw.m[4] = Tpw.at<float>(0,1);
    glTpw.m[5] = Tpw.at<float>(1,1);
    glTpw.m[6] = Tpw.at<float>(2,1);
    glTpw.m[7]  = 0.0;

    glTpw.m[8] = Tpw.at<float>(0,2);
    glTpw.m[9] = Tpw.at<float>(1,2);
    glTpw.m[10] = Tpw.at<float>(2,2);
    glTpw.m[11]  = 0.0;

    glTpw.m[12] = Tpw.at<float>(0,3);
    glTpw.m[13] = Tpw.at<float>(1,3);
    glTpw.m[14] = Tpw.at<float>(2,3);
    glTpw.m[15]  = 1.0;

}

Plane::Plane(const float &nx, const float &ny, const float &nz, const float &ox, const float &oy, const float &oz)
{
    n = (cv::Mat_<float>(3,1)<<nx,ny,nz);
    o = (cv::Mat_<float>(3,1)<<ox,oy,oz);

    cv::Mat up = (cv::Mat_<float>(3,1) << 0.0f, 1.0f, 0.0f);

    cv::Mat v = up.cross(n);
    const float s = cv::norm(v);
    const float c = up.dot(n);
    const float a = atan2(s,c);
    //特殊矩阵初始化，4*4的对角矩阵
    Tpw = cv::Mat::eye(4,4,CV_32F);
    const float rang = -3.14f/2+((float)rand()/RAND_MAX)*3.14f;
    cout << rang;
    //Pc=TPw=R*Pw+t
    Tpw.rowRange(0,3).colRange(0,3) = ExpSO3(v*a/s)*ExpSO3(up*rang);
    o.copyTo(Tpw.col(3).rowRange(0,3));

    glTpw.m[0] = Tpw.at<float>(0,0);
    glTpw.m[1] = Tpw.at<float>(1,0);
    glTpw.m[2] = Tpw.at<float>(2,0);
    glTpw.m[3]  = 0.0;

    glTpw.m[4] = Tpw.at<float>(0,1);
    glTpw.m[5] = Tpw.at<float>(1,1);
    glTpw.m[6] = Tpw.at<float>(2,1);
    glTpw.m[7]  = 0.0;

    glTpw.m[8] = Tpw.at<float>(0,2);
    glTpw.m[9] = Tpw.at<float>(1,2);
    glTpw.m[10] = Tpw.at<float>(2,2);
    glTpw.m[11]  = 0.0;

    glTpw.m[12] = Tpw.at<float>(0,3);
    glTpw.m[13] = Tpw.at<float>(1,3);
    glTpw.m[14] = Tpw.at<float>(2,3);
    glTpw.m[15]  = 1.0;
}


}
