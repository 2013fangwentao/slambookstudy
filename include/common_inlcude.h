#ifndef COMMON_INCLUDE_H_
#define COMMON_INCLUDE_H_

//for Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

//for Sophus
#include <sophus/se3.h>
using Sophus::SE3;

//for cv
#include <opencv2/core/core.hpp>

// std
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>

#endif