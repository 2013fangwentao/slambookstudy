#ifndef VISUALODOMETRY_H_
#define VISUALODOMETRY_H_
#include "common_inlcude.h"
#include "fMap.h"
#include "fFrame.h"

namespace myslam
{

template <typename T>
T max(T t1, T t2)
{
    return t1 > t2 ? t1 : t2;
}

class fVisualOdometry
{
  public:
    using Ptr = std::shared_ptr<fVisualOdometry>;
    enum VOState
    {
        INITIALIZING = -1,
        OK = 0,
        LOST
    };

    VOState vo_state_;
    fMap::Ptr map_;

    fFrame::Ptr ref_frame_;
    fFrame::Ptr curr_frame_;

    cv::Ptr<cv::ORB> orb_;                     //orb detector and computer
    std::vector<cv::Point3f> pts_3d_ref_;      //3d points in reference frame
    std::vector<cv::KeyPoint> keypoints_curr_; //keypoints in current frame
    cv::Mat descriptors_curr_;                 //descriptors in current frame
    cv::Mat descriptors_ref_;                  //descriptors in reference frame

    std::vector<cv::DMatch> feature_matches_;
    cv::FlannBasedMatcher matcher_flann_;
    std::vector<fMapPoint::Ptr> match_3dpts_;   // matched 3d points
    std::vector<int> match_2dkp_index_;        // matched 2d pixels (index of kp_curr)


//    SE3 T_c_r_estimated_;
    SE3 T_c_w_estimated_;
    int num_inliers_;
    int num_lost_;

    int num_of_features_;
    double scale_factor_;
    int level_pyramid_;
    float match_ratio_;
    int max_num_lost_;
    int min_inliers_;

    double key_frame_min_rot;
    double key_frame_min_trans;
    double map_point_erease_ratio_;

  public:
    fVisualOdometry();
    ~fVisualOdometry();

    bool addFrame(fFrame::Ptr frame);

  protected:
    void extractKeyPoint();
    void computeDescriptors();
    void featureMatching();
    void poseEstimationPnP();
    void setRef3DPoints();
    void optimizeMap();

    void addKeyFrame();
    bool checkEstimatedPose();
    bool checkKeyFrame();
    void addMapPoints();

    double getViewAngle(fFrame::Ptr frame, fMapPoint::Ptr point);

};

} // namespace myslam

#endif //VISUALODOMETRY_H_