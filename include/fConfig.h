#ifndef CONFIG_H_
#define CONFIG_H_

#include "common_inlcude.h"

namespace myslam
{

class fConfig
{
private:
    static std::shared_ptr<fConfig> config_;
    cv::FileStorage file_;
    fConfig(){};
    fConfig(const fConfig&);
    fConfig& operator =(const fConfig&);

public:
    ~fConfig();
    static void setParameterFile(const std::string& filename);

    template< typename T>
    static T get (const std::string& key)
    {
        return T(fConfig::config_->file_[key]);
    }
};

}

#endif //CONFIG_H_