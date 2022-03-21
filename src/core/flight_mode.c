#include <flight_mode.h>


bool mode_needs_mocap(flight_mode_t mode){
    if (mode == AUTONOMOUS || mode == LOITER || mode == LOITER_RSP || mode == ALT_HOLD)
    {
        return true;
    }
    return false;
}

bool mode_needs_lcm(flight_mode_t mode) {
    if (mode == SENTRY || mode == FOLLOW_ME)
    {
        return true;
    }
    return false;
}