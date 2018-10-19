#include "fMapPoint.h"

namespace myslam
{

fMapPoint::fMapPoint()
:id_(-1),pos_(Vector3d(0,0,0)),norm_(Vector3d(0,0,0)),observed_times_(0),correct_times_(0)
{
    
}

fMapPoint::fMapPoint( long id, Vector3d position, Vector3d norm )
:id_(id), pos_(position), norm_(norm), observed_times_(0), correct_times_(0)
{

}

fMapPoint::Ptr fMapPoint::createfMapPoint()
{
    static long factory_id=0;
    return fMapPoint::Ptr(
        new fMapPoint(factory_id++,Vector3d(0,0,0),Vector3d(0,0,0))
    );
}

}