#ifndef MAPPOINT_H_
#define MAPPOINT_H_

#include "common_inlcude.h"

namespace myslam
{


class fFrame;

class fMapPoint
{
public:
    using Ptr = std::shared_ptr<fMapPoint> ;
    unsigned long                    id_;//ID
    Vector3d                         pos_;//Position in world
    Vector3d                         norm_;//Normal of viewing direction
    cv::Mat                          descriptor_; //Descriptor for matching 
    int                              observed_times_;// being observed by feature matching algo
    int                              correct_times_; // being an inliner in pose estimation

public: 
    fMapPoint();
    fMapPoint( long id,Vector3d position, Vector3d norm );


    static fMapPoint::Ptr createfMapPoint();
};



}


#endif //MAPPOINT_H_