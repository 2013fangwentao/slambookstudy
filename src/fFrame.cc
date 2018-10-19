#include "fFrame.h"

namespace myslam
{
fFrame::fFrame():id_(-1),time_stamp_(-1),camera_(nullptr)
{

}
fFrame::fFrame(long id,double time_stamp,SE3 T_c_w,
               Camera::Ptr camera,cv::Mat color,cv::Mat depth):
               id_(id),time_stamp_(time_stamp),T_c_w_(T_c_w),
               camera_(camera),color_(color),depth_(depth)
{

}

fFrame::~fFrame()
{

}

fFrame::Ptr fFrame::createFrame()
{
    static long factory_id = 0;      
    return fFrame::Ptr(new fFrame(factory_id++));
}

double fFrame::findDepth(const cv::KeyPoint& kp)
{
    int x=cvRound(kp.pt.x);
    int y=cvRound(kp.pt.y);
    ushort d = depth_.ptr<ushort>(y)[x];
    if(d!=0)
    {
        return double(d)/camera_->depth_scale_;
    }
    else
    {
        //check the nearby points
        int dx[4] = {-1,0,1,0};
        int dy[4] = {0,-1,0,1};
        for ( int i=0; i<4; i++ )
        {
            d = depth_.ptr<ushort>( y+dy[i] )[x+dx[i]];
            if ( d!=0 )
            {
                return double(d)/camera_->depth_scale_;
            }
        }
    }
    return -1.0;
}

Vector3d fFrame::getCamCenter() const
{
     return T_c_w_.inverse().translation();
}

bool fFrame::isInFrame(const Vector3d& pt_world)
{
    Vector3d p_cam = camera_->world2camera(pt_world,T_c_w_);
    if(p_cam(2,0)<0)
        return false;
    Vector2d pixel = camera_->world2pixel(pt_world,T_c_w_);
    return pixel(0,0)>0&&pixel(1,0)>0
           &&pixel(0,0)<color_.cols
           &&pixel(1,0)<color_.rows; 
}


}