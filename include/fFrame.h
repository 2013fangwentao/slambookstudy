#ifndef FRAME_H_
#define FRAME_H_

#include "common_inlcude.h"
#include "fCamera.h"

namespace myslam
{

class fMapPoint;
class fFrame
{
public: 
    using Ptr = std::shared_ptr<fFrame>;
    unsigned long            id_;             //id of frame
    double                   time_stamp_;     //the time of the frame
    SE3                      T_c_w_;          //transform from world to camera
    Camera::Ptr              camera_;         //Pinhole RGBD Camera model
    cv::Mat                  color_,depth_;   //color and depth image

public:
    fFrame();
    fFrame(long id,double time_stamp = 0.0,SE3 T_c_w=SE3(),
           Camera::Ptr camera_=nullptr,cv::Mat color=cv::Mat(),
           cv::Mat depth =cv::Mat());
    ~fFrame();

    //factory function 
    static fFrame::Ptr createFrame();

    //find the depth in depth map
    double findDepth(const cv::KeyPoint& kp);

    //Get Camera Center
    Vector3d getCamCenter() const;

    // check if a point is in this frame
    bool isInFrame(const Vector3d& pt_world);

};


}


#endif //FRAME_H_