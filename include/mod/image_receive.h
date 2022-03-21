#ifndef __IMAGE_RECEIVE__
#define __IMAGE_RECEIVE__

#include <stdint.h>

typedef struct Image_data_t 
{
    double u;
    double v;
    double range;
    double bearing;
} Image_data_t;

extern Image_data_t Image_data;

#endif
