#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include "esp_camera.h"
#include "types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Extern declaration for global socket defined in main
extern int sock;

extern target_box *shared_box;
extern SemaphoreHandle_t box_mutex;

extern TaskHandle_t image_proc_handle;
extern TaskHandle_t image_send_handle;
extern TaskHandle_t tcp_server_handle;

// ===================
// Function Prototypes
// ===================

/**
 * @brief Task to initialize a TCP socket and accept one client.
 * 
 *  This function sets up a TCP server that listens to a defined port.
 *  Accepts a single incoming client connection, and assigns the client
 *  to the global sock variable. In case the client disconnects, resets the socket and waits for a new client.
 * 
 * @param pvParameters Needed for FreeRTOS tasks.
 * 
 */
void tcp_server_task(void *pvParameters);

/**
 * @brief Function used to send binary image data over TCP.
 * 
 * @param fb The frame buffer to be sent.
 * 
 * @return Returns 0 if send successful, -1 otherwise.
 */
int tcp_send_frame(const camera_fb_t *fb);

/**
 * @brief  Function used to send the video dimensions to the client. To be used before tcp_send.
 *  Send must succeed, else stream cannot be started.
 * 
 * @param fb The frame buffer to be sent.
 * @return Returns 0 if successful, -1 otherwise.
 */
int tcp_send_dims(const camera_fb_t *fb);

/**
 * @brief Sends the target box to the client. 
 * 
 * 
 * @param box Reference to the target box.
 * @return int Returns 0 if successful, -1 otherwise.
 */
int tcp_send_box(const target_box *box);

/**
 * @brief Sends the disconnect signal to the client, and shuts down the socket.
 * 
 * @param sock Pointer to the socket to be closed
 * @return int Returns 0 if successful, -1 otherwise.
 */
int disconnect(int *sock);

#endif // TCP_SERVER_H