/**
 * <printf_manager.h>
 *
 * @brief   Functions to start and stop the printf mnaager which is a
 * separate thread printing data to the console for debugging.
 *
 * @addtogroup PrintfManager
 * @{
 */

#ifndef __SOCKET_MANAGER__
#define __SOCKET_MANAGER__

#include <flight_mode.h>

/**
 * @brief   Start the printf_manager thread which should be the only thing
 * printing to the screen besides error messages from other threads.
 *
 * @return  0 on success, -1 on failure
 */
int socketserver_init(void);

/**
 * @brief   Waits for the printf manager thread to exit.
 *
 * @return  0 on clean exit, -1 on exit time out/force close
 */
int socketserver_cleanup(void);


#endif /* __SOCKET_MANAGER__ */

/* @} end group PrintfManager */
