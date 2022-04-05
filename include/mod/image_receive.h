#ifndef __IMAGE_RECEIVE__
#define __IMAGE_RECEIVE__

#include <stdint.h>

typedef struct object_observation_t 
{
    uint64_t object_observation_time_ns;
    double u;
    double v;
    double range;
    double bearing;
} object_observation_t;

typedef struct visual_odometry_t
{ 
    uint64_t visual_odometry_time_ns;
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;

} visual_odometry_t;


extern object_observation_t object_observation;
extern visual_odometry_t visual_odometry;

#endif
