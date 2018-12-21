#include "fMapPoint.h"

namespace myslam
{
unsigned long fMapPoint::factory_id_ = 0;

fMapPoint::fMapPoint()
    : id_(-1), pos_(Vector3d(0, 0, 0)), norm_(Vector3d(0, 0, 0)), good_(true), visible_times_(0), matched_times_(0)
{
}

fMapPoint::fMapPoint(long id, const Vector3d &position, const Vector3d &norm, fFrame *frame, const cv::Mat &descriptor)
    : id_(id), pos_(position), norm_(norm), visible_times_(0), matched_times_(0), good_(true), descriptor_(descriptor)
{
    observed_frames_.push_back(frame);
}

fMapPoint::Ptr fMapPoint::createfMapPoint()
{
    return fMapPoint::Ptr(new fMapPoint(factory_id_++, Vector3d(0, 0, 0), Vector3d(0, 0, 0)));
}

fMapPoint::Ptr fMapPoint::createfMapPoint(const Vector3d &pos_world,
                                          const Vector3d &norm,
                                          const cv::Mat &descriptor,
                                          fFrame *frame)

{
    return fMapPoint::Ptr(new fMapPoint(factory_id_++, pos_world, norm, frame, descriptor));
}

} // namespace myslam