#include "fConfig.h"
#include "stdio.h"
namespace myslam
{

std::shared_ptr<fConfig> fConfig::config_ = nullptr;

void fConfig::setParameterFile(const std::string& filename)
{
    if(config_==nullptr)
        config_ = std::shared_ptr<fConfig>(new fConfig);
    config_->file_ = cv::FileStorage(filename.c_str(),cv::FileStorage::READ);
    if(config_->file_.isOpened() == false)
    {
        std::cerr<<"parameter file "<<filename<<" does not exist."<<std::endl;
        config_->file_.release();
        return ;
    }
}

fConfig::~fConfig()
{
    if (file_.isOpened())
        file_.release();
}

}