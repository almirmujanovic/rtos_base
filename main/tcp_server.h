#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief Task that handles incoming TCP connections and communication with PC
 * 
 * This function is meant to be used as a FreeRTOS task and should be launched via xTaskCreate.
 * It sets up a server socket and handles data exchange with the PC over WiFi.
 * 
 * @param pvParameters Optional pointer to task parameters (set to NULL if not used)
 */
void tcp_server_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // TCP_SERVER_H
