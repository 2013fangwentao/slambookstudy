#ifndef MAPPOINT_H_
#define MAPPOINT_H_

#include "common_inlcude.h"
#include <list>
namespace myslam
{

class fFrame;

class fMapPoint
{
  public:
    using Ptr = std::shared_ptr<fMapPoint>;
    unsigned long id_;                //ID
    Vector3d pos_;                    //Position in world
    Vector3d norm_;                   //Normal of viewing direction
    cv::Mat descriptor_;              //Descriptor for matching
    static unsigned long factory_id_; //factory ID
    bool good_;                       //wheter a good point
    std::list<fFrame *> observed_frames_;  //Key-frames that can observe this point
    int matched_times_;               // being an inliner in pose estimation
    int visible_times_;               // being visible in current frame
                                      // int observed_times_;              // being observed by feature matching algo
                                      // int correct_times_;               // being an inliner in pose estimation

  public:
    fMapPoint();
    fMapPoint(long id, const Vector3d &position,
              const Vector3d &norm, fFrame *frame = nullptr, const cv::Mat &descriptor = cv::Mat());

    inline cv::Point3f getPositionCV() const
    {
        return cv::Point3f(pos_(0, 0), pos_(1, 0), pos_(2, 0));
    }
    static fMapPoint::Ptr createfMapPoint();
    static fMapPoint::Ptr createfMapPoint(
        const Vector3d &pos_world,
        const Vector3d &norm_,
        const cv::Mat &descriptor,
        fFrame *frame);
};

} // namespace myslam

#endif //MAPPOINT_H_