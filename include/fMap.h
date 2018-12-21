#ifndef MAP_H_
#define MAP_H_

#include <unordered_map>
#include "common_inlcude.h"
#include "fMapPoint.h"
#include "fFrame.h"

namespace myslam
{

class fMap
{
  public:
    using Ptr = std::shared_ptr<fMap>;
    std::unordered_map<unsigned long, fMapPoint::Ptr> map_points_; //all landmarks
    std::unordered_map<unsigned long, fFrame::Ptr> key_frames_;    //all key frames
    fMap() {}

    void insertKeyFrame(fFrame::Ptr frame);
    void insertMapPoint(fMapPoint::Ptr map_point);
};

} // namespace myslam

#endif //MAP_H_