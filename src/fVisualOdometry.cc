#include "fVisualOdometry.h"
#include "fConfig.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "sophus/so3.h"
using namespace std;
using namespace cv;
using namespace Sophus;
namespace myslam
{

fVisualOdometry::fVisualOdometry():
vo_state_(INITIALIZING),ref_frame_(nullptr),curr_frame_(nullptr),map_(new fMap),num_lost_(0),num_inliers_(0)
{
    num_of_features_      = fConfig::get<int>("number_of_features");
    scale_factor_         = fConfig::get<double>("scale_factor");
    level_pyramid_        = fConfig::get<int> ("level_pyramid");
    match_ratio_          = fConfig::get<float> ("match_ratio");
    max_num_lost_         = fConfig::get<float> ("max_num_lost");
    min_inliers_          = fConfig::get<int> ("min_liners");
    key_frame_min_rot     = fConfig::get<double>("keyframe_rotation");
    key_frame_min_trans   = fConfig::get<double> ( "keyframe_translation" );
    orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
}

fVisualOdometry::~fVisualOdometry(){}

bool fVisualOdometry::addFrame(fFrame::Ptr frame)
{
    switch(vo_state_)
    {
        case INITIALIZING:{
            vo_state_ = OK;
            curr_frame_ = ref_frame_ = frame;
            map_->insertKeyFrame(frame);
            extractKeyPoint();
            computeDescriptors();
            setRef3DPoints();
            break;
        }
        case OK:{
            curr_frame_ = frame;
            extractKeyPoint();
            computeDescriptors();
            featureMatching();
            poseEstimationPnP();
            if(checkEstimatedPose()==true)
            {
                curr_frame_->T_c_w_ = T_c_r_estimated_ * ref_frame_->T_c_w_;
                ref_frame_ = curr_frame_;
                setRef3DPoints();
                num_lost_ = 0;
                if(checkKeyFrame() == true)
                {
                    addKeyFrame();
                }
            }
            else
            {
                num_lost_++;
                if ( num_lost_ > max_num_lost_ )
                {
                    vo_state_ = LOST;
                }
                return false;
            }
            break;
        }
        case LOST:{
            std::cout<<"vo has lost."<<std::endl;
            break;
        }
    }
    return true;
}

void fVisualOdometry::extractKeyPoint()
{
    orb_->detect(curr_frame_->color_,keypoints_curr_);
}

void fVisualOdometry::computeDescriptors()
{
    orb_->compute ( curr_frame_->color_, keypoints_curr_, descriptors_curr_ );
}

void fVisualOdometry::featureMatching()
{
    // match desp_ref and desp_curr, use OpenCV's brute force match 
    std::vector<cv::DMatch> matches;
    cv::BFMatcher matcher ( cv::NORM_HAMMING );
    matcher.match ( descriptors_ref_, descriptors_curr_, matches );
    // select the best matches
    float min_dis = std::min_element (
                        matches.begin(), matches.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
        return m1.distance < m2.distance;
    } )->distance;

    feature_matches_.clear();
    for ( cv::DMatch& m : matches )
    {
        if ( m.distance <max<float> ( min_dis*match_ratio_, 30.0 ) )
        {
            feature_matches_.push_back(m);
        }
    }
    std::cout<<"good matches: "<<feature_matches_.size()<<std::endl;
}

void fVisualOdometry::setRef3DPoints()
{
    // select the features with depth measurements 
    pts_3d_ref_.clear();
    descriptors_ref_ = cv::Mat();
    for ( size_t i=0; i<keypoints_curr_.size(); i++ )
    {
        double d = ref_frame_->findDepth(keypoints_curr_[i]);               
        if ( d > 0)
        {
            Vector3d p_cam = ref_frame_->camera_->pixel2camera(
                Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), d
            );
            pts_3d_ref_.push_back( cv::Point3f( p_cam(0,0), p_cam(1,0), p_cam(2,0) ));
            descriptors_ref_.push_back(descriptors_curr_.row(i));
        }
    }
}

void fVisualOdometry::poseEstimationPnP()
{
    // construct the 3d 2d observations
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;
    
    for ( cv::DMatch m:feature_matches_ )
    {
        pts3d.push_back( pts_3d_ref_[m.queryIdx] );
        pts2d.push_back( keypoints_curr_[m.trainIdx].pt );
    }
    
    Mat K = ( cv::Mat_<double>(3,3)<<
        ref_frame_->camera_->fx_, 0, ref_frame_->camera_->cx_,
        0, ref_frame_->camera_->fy_, ref_frame_->camera_->cy_,
        0,0,1
    );
    Mat rvec, tvec, inliers;
    cv::solvePnPRansac( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );
    num_inliers_ = inliers.rows;
    cout<<"pnp inliers: "<<num_inliers_<<endl;
    T_c_r_estimated_ = SE3(
        SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)), 
        Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
    );
}

bool fVisualOdometry::checkEstimatedPose()
{
    // check if the estimated pose is good
    if ( num_inliers_ < min_inliers_ )
    {
        cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
        return false;
    }
    // if the motion is too large, it is probably wrong
    Sophus::Vector6d d = T_c_r_estimated_.log();
    if ( d.norm() > 5.0 )
    {
        cout<<"reject because motion is too large: "<<d.norm()<<endl;
        return false;
    }
    return true;
}

bool fVisualOdometry::checkKeyFrame()
{
    Sophus::Vector6d d = T_c_r_estimated_.log();
    Vector3d trans = d.head<3>();
    Vector3d rot = d.tail<3>();
    if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
        return true;
    return false;
}

void fVisualOdometry::addKeyFrame()
{
    cout<<"adding a key-frame"<<endl;
    map_->insertKeyFrame ( curr_frame_ );
}




}//namespace myslam
