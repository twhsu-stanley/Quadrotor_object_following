/**
 * <thrust_map.h>
 *
 * @brief   Functions to start and stop the printf manager which is a
 * separate thread printing data to the console for debugging.
 *
 * @addtogroup ThrustMap
 * @{
 */

#ifndef __THRUST_MAP__
#define __THRUST_MAP__

/**
 * enum thrust_map_t
 *
 * the user may select from the following preconfigured thrust maps
 */
typedef enum thrust_map_t
{
    LINEAR_MAP,
    MN1806_1400KV_4S,
    F20_2300KV_2S,
    RX2206_4S,
    AIR2213_3S,
    QUADLAB_MAP
} thrust_map_t;

/**
 * @brief      Check the thrust map for validity and populate data arrays.
 *
 * @return     0 on success, -1 on failure
 */
int thrust_map_init(thrust_map_t map);

/**
 * @brief      Corrects the motor signal m for non-linear thrust curve in place.
 *
 *
 * @param[in]  m     thrust input, must be between 0 and 1 inclusive
 *
 * @return     motor signal value on success, -1 on error
 */
double map_motor_signal(double m);

#endif  // __THRUST_MAP__

/* @} end group ThrustMap */