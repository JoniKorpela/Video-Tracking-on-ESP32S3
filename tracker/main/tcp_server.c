#include "tcp_server.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_camera.h"

#include "lwip/sockets.h"
#include <lwip/netdb.h>

#include "esp_timer.h"

#include "types.h"

#define PORT 8001

static const char *TAG = "TCP_SERVER";

static void close_socket(int *sock)
{
    if (*sock >= 0)
    {
        close(*sock);
        *sock = -1;
    }
}

static int tcp_receive()
{
    if (sock < 0)
        return -1;

    uint8_t header_data;
    int ret = recv(sock, &header_data, sizeof(header_data), 0);
    if (ret <= 0)
    {
        ESP_LOGE(TAG, "errno %d", errno);
        return -1;
    }

    if (header_data == CONTROL_MESSAGE)
    {
        uint8_t control_cmd;
        ret = recv(sock, &control_cmd, 1, 0);
        if (ret <= 0)
        {
            ESP_LOGE(TAG, "Failed to receive control data: errno %d", errno);
            close_socket(&sock);
            return -1;
        }

        if (control_cmd == STOP_TRACKING)
        {
            xTaskNotify(image_proc_handle, EVENT_STOP_TRACKING, eSetBits);
        }
        else if (control_cmd == DISCONNECT)
        {
            xTaskNotify(image_proc_handle, EVENT_DISCONNECT, eSetBits);
        }
        return 0;
    }
    else if (header_data == BOX_MESSAGE)
    {
        target_box box_data;
        uint8_t *buf = (uint8_t *)&box_data;
        size_t expected = sizeof(target_box);
        size_t received = 0;

        while (received < expected)
        {
            ret = recv(sock, buf + received, expected - received, 0);
            if (ret <= 0)
            {
                ESP_LOGE(TAG, "Failed to receive target box: errno %d", errno);
                close_socket(&sock);
                return -1;
            }
            received += ret;
        }

        // Convert from network byte order
        box_data.x_min = ntohs(box_data.x_min);
        box_data.y_min = ntohs(box_data.y_min);
        box_data.x_max = ntohs(box_data.x_max);
        box_data.y_max = ntohs(box_data.y_max);

        xSemaphoreTake(box_mutex, portMAX_DELAY);
        *shared_box = box_data;
        xSemaphoreGive(box_mutex);

        xTaskNotify(image_proc_handle, EVENT_START_TRACKING, eSetBits);
        return 0;
    }

    return -1;
}

int disconnect(int *sock)
{
    uint8_t header = CONTROL_MESSAGE;
    uint8_t command = DISCONNECT;
    int sent = send(*sock, &header, sizeof(header), 0);
    if (sent != sizeof(header))
    {
        ESP_LOGE(TAG, "Error sending disconnect command header via TCP: errno %d", errno);
        return -1;
    }
    sent = send(*sock, &command, sizeof(command), 0);
    if (sent != sizeof(command))
    {
        ESP_LOGE(TAG, "Error sending disconnect command data via TCP: errno %d", errno);
        return -1;
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    close_socket(sock);
    return 0;
}

void tcp_server_task(void *pvParameters)
{
    struct sockaddr_in server_addr; // Server's address
    struct sockaddr_in client_addr; // Client's address

    // Create socket
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0)
    {
        ESP_LOGE(TAG, "Unnable to create socket: errno %d", errno);
        close_socket(&listen_sock);
        vTaskDelete(NULL);
        return;
    }

    // Allow reuse of address in case of quick shutdown/startup
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Bind socket to a port
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(PORT);

    int err = bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (err != 0)
    {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close_socket(&listen_sock);
        vTaskDelete(NULL);
        return;
    }

    // Start listening
    err = listen(listen_sock, 1);
    if (err < 0)
    {
        ESP_LOGE(TAG, "Error occured during listen: errno %d", errno);
        close_socket(&listen_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "TCP socket listening on port %d", PORT);

    while (1)
    {
        // Accept one client
        socklen_t addr_len = sizeof(client_addr);
        int client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
        if (client_sock < 0)
        {
            ESP_LOGE(TAG, "Failed to accept connection: errno %d", errno);
            close_socket(&listen_sock);
            vTaskDelete(NULL);
            return;
        }

        // Assing the accepted client to the global socket
        sock = client_sock;

        // TODO: uncomment to disable nagle's algorithm
        int flag = 1;
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

        ESP_LOGI(TAG, "Client connected");

        // Notify image proc task that stream can start
        xTaskNotify(image_proc_handle, EVENT_CONNECTED, eSetBits);

        while (sock >= 0)
        {
            if (tcp_receive() < 0)
            {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10)); 
        }
    }
}

int tcp_send_frame(const camera_fb_t *fb)
{
    if (sock >= 0)
    {
        size_t len = fb->len;
        uint8_t *data = fb->buf;
        uint8_t header = FRAME_MESSAGE;
        int sent = send(sock, &header, sizeof(header), 0);
        if (sent != sizeof(header))
        {
            ESP_LOGE(TAG, "Error sending target frame data via TCP: errno %d", errno);
            return -1;
        }
        while (len > 0)
        {
            int sent = send(sock, data, len, 0);
            if (sent <= 0)
            {
                ESP_LOGE(TAG, "Error during sending frame data via TCP: errno %d", errno);
                return -1;
            }
            len -= sent;
            data += sent;
        }
        return 0;
    }
    else
        return -1;
}

int tcp_send_dims(const camera_fb_t *fb)
{
    if (sock >= 0)
    {
        ESP_LOGI(TAG, "sending dimensions to client");
        // Send video dimensions if this is the first frame.

        uint8_t header = DIM_MESSAGE;
        int sent = send(sock, &header, sizeof(header), 0);
        if (sent != sizeof(header))
        {
            ESP_LOGE(TAG, "Error sending target box header via TCP: errno %d", errno);
            return -1;
        }

        uint16_t width = htons(fb->width);
        uint16_t height = htons(fb->height);
        uint16_t dims[2] = {width, height};
        sent = send(sock, &dims, sizeof(dims), 0);
        if (sent != sizeof(dims))
        {
            ESP_LOGE(TAG, "Error sending dimensions via TCP. Closing socket: errno %d", errno);
            close_socket(&sock);
            return -1;
        }
        return 0;
    }
    else
        return -1;
}
int tcp_send_box(const target_box *box)
{
    if (sock >= 0)
    {
        uint16_t net_box[4] = {
            htons(box->x_min),
            htons(box->y_min),
            htons(box->x_max),
            htons(box->y_max)};
        uint8_t header = BOX_MESSAGE;

        int sent = send(sock, &header, sizeof(header), 0);
        if (sent != sizeof(header))
        {
            ESP_LOGE(TAG, "Error sending target box header via TCP: errno %d", errno);
            return -1;
        }

        sent = send(sock, net_box, sizeof(net_box), 0);
        if (sent != sizeof(net_box))
        {
            ESP_LOGE(TAG, "Error sending target box data via TCP: errno %d", errno);
            return -1;
        }
        return 0;
    }
    else
        return -1;
}