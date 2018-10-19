#include "fMap.h"
#include "fConfig.h"
#include "stdio.h"

using namespace myslam;

int main(int argc,char** argv)
{
    fConfig::setParameterFile("./default.yaml");

    double fx = fConfig::get<double>("camera.fx");
    fMap map;
    return 0;
}