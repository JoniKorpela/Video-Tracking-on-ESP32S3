#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "tcp_server.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "camera_pins.h"

#include "cred.h"
#include "tracker.h"

TaskHandle_t image_proc_handle = NULL;
TaskHandle_t image_send_handle = NULL;
TaskHandle_t tcp_server_handle = NULL;

static uint8_t shared_fb_buf[MAX_FRAME_SIZE];
static camera_fb_t shared_fb_instance;
static target_box shared_box_instance;

static camera_fb_t *shared_fb = NULL;
static SemaphoreHandle_t fb_mutex;
static SemaphoreHandle_t tcp_mutex;

target_box *shared_box = NULL;
SemaphoreHandle_t box_mutex = NULL;

int sock = -1;

static const char *TAG = "MAIN";

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        ESP_LOGI(TAG, "Wi-Fi STA started, connecting...");
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    {
        ESP_LOGI(TAG, "Wi-Fi STA connected to AP");
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGI(TAG, "Wi-Fi STA disconnected, retrying...");
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_LOST_IP)
    {
        ESP_LOGI(TAG, "Lost IP address.");
    }
}

static void init_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_LOST_IP, &wifi_event_handler, NULL, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID_STA,
            .password = WIFI_PASS_STA,
            .scan_method = WIFI_FAST_SCAN,
            .bssid_set = false,
            .channel = 0,
            .listen_interval = 0,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false,
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static const camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sccb_sda = SIOD_GPIO_NUM,
    .pin_sccb_scl = SIOC_GPIO_NUM,
    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,
    .xclk_freq_hz = 20000000,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .pixel_format = PIXFORMAT_GRAYSCALE,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count = 1,
    .grab_mode = CAMERA_GRAB_LATEST};

void image_proc_task(void *pvParameters)
{
    uint32_t event_val;
    camera_fb_t *fb = NULL;

    // TIMING ACCU
    static int64_t fb_return_accu = 0;
    static int64_t fb_get_accu = 0;
    static int64_t memcpy_accu = 0;
    static int64_t tracker_accu = 0;
    static int64_t loop_total_accu = 0;
    static float elapsed_time = 0;
    static float prev_time = 0;
    static uint32_t frame_count = 0;
    // TIMING ACCU

    enum TASK_STATE
    {
        DISCONNECTED,
        FIRST_FRAME,
        STREAM_VIDEO,
        TRACK_TARGET,
    };

    enum TASK_STATE task_state = DISCONNECTED;

    while (1)
    {
        xTaskNotifyWait(0, EVENT_CONNECTED, &event_val, portMAX_DELAY);
        task_state = FIRST_FRAME;

        while (1)
        {
            int64_t loop_start = esp_timer_get_time();
            if (sock < 0)
            {
                task_state = DISCONNECTED;
                tracker(NULL, shared_box, true);
                break;
            }

            BaseType_t notified = xTaskNotifyWait(0, 0xFFFFFFFF, &event_val, 0);
            if (notified == pdTRUE)
            {
                if (event_val & EVENT_STOP_TRACKING)
                {
                    tracker(NULL, shared_box, true);
                    task_state = STREAM_VIDEO;
                }
                if (event_val & EVENT_START_TRACKING)
                {
                    task_state = TRACK_TARGET;
                }
                if (event_val & EVENT_DISCONNECT)
                {
                    task_state = DISCONNECTED;
                }
            }

            if (task_state == DISCONNECTED)
            {
                xSemaphoreTake(tcp_mutex, portMAX_DELAY);
                disconnect(&sock);
                tracker(NULL, shared_box, true);
                xSemaphoreGive(tcp_mutex);
                break;
            }
            int64_t fb_get_start = esp_timer_get_time();
            fb = esp_camera_fb_get();
            int64_t memcpy_start = esp_timer_get_time();

            if (!fb || !fb->buf)
                continue;

            if (xSemaphoreTake(fb_mutex, portMAX_DELAY))
            {
                memcpy(shared_fb->buf, fb->buf, fb->len);
                shared_fb->format = fb->format;
                shared_fb->height = fb->height;
                shared_fb->len = fb->len;
                shared_fb->timestamp = fb->timestamp;
                shared_fb->width = fb->width;

                if (task_state == FIRST_FRAME)
                {
                    xTaskNotify(image_send_handle, EVENT_FIRST_FRAME, eSetBits);
                    task_state = STREAM_VIDEO;
                }
                else
                {
                    xTaskNotify(image_send_handle, EVENT_NEW_FRAME, eSetBits);
                }
                xSemaphoreGive(fb_mutex);
            }
            int64_t track_target_start = esp_timer_get_time();

            if (task_state == TRACK_TARGET)
            {
                xSemaphoreTake(box_mutex, portMAX_DELAY);
                tracker(fb, shared_box, false);
                xSemaphoreGive(box_mutex);
                xTaskNotify(image_send_handle, EVENT_NEW_BOX, eSetBits);
            }
            int64_t fb_return_start = esp_timer_get_time();

            float curr_time = fb->timestamp.tv_sec + fb->timestamp.tv_usec / 1e6;

            esp_camera_fb_return(fb);
            fb = NULL;

            int64_t loop_end = esp_timer_get_time();

            // TIMING
            float dt = curr_time - prev_time;
            prev_time = curr_time;
            elapsed_time += dt;
            frame_count++;

            fb_get_accu += memcpy_start - fb_get_start;
            memcpy_accu += track_target_start - memcpy_start;
            tracker_accu += fb_return_start - track_target_start;
            fb_return_accu += loop_end - fb_return_start;
            loop_total_accu += loop_end - loop_start;
            if (elapsed_time >= 10.0f)
            {
                printf(
                    "\n=== Image Proc Task Averages (last %.1f s, %lu frames) ===\n"
                    "fb_get:       %llu us (%.3f ms)\n"
                    "memcpy+mutex: %llu us (%.3f ms)\n"
                    "tracker:      %llu us (%.3f ms)\n"
                    "fb_return:    %llu us (%.3f ms)\n"
                    "loop total:   %llu us (%.3f ms)\n\n",
                    elapsed_time, frame_count,
                    (unsigned long long)(fb_get_accu / frame_count), (fb_get_accu / frame_count) / 1000.0,
                    (unsigned long long)(memcpy_accu / frame_count), (memcpy_accu / frame_count) / 1000.0,
                    (unsigned long long)(tracker_accu / frame_count), (tracker_accu / frame_count) / 1000.0,
                    (unsigned long long)(fb_return_accu / frame_count), (fb_return_accu / frame_count) / 1000.0,
                    (unsigned long long)(loop_total_accu / frame_count), (loop_total_accu / frame_count) / 1000.0);

                // reset accumulators
                fb_get_accu = 0;
                memcpy_accu = 0;
                tracker_accu = 0;
                fb_return_accu = 0;
                loop_total_accu = 0;
                frame_count = 0;
                elapsed_time = 0.0f;
            }
        }
    }
}
void image_send_task(void *pvParameters)
{
    uint32_t event_val;

    // TIMING ACCU
    int64_t dims_accu = 0;
    int64_t frame_accu = 0;
    int64_t box_accu = 0;
    int64_t loop_total_accu = 0;

    // COUNTERS
    uint32_t dims_count = 0;
    uint32_t frame_count = 0;
    uint32_t box_count = 0;
    uint32_t loop_count = 0;

    float elapsed_time = 0;

    while (1)
    {
        int64_t loop_start = esp_timer_get_time();

        xTaskNotifyWait(0, EVENT_NEW_FRAME | EVENT_NEW_BOX | EVENT_FIRST_FRAME, &event_val, portMAX_DELAY);

        // FIRST FRAME
        if (event_val & EVENT_FIRST_FRAME)
        {
            int64_t a = esp_timer_get_time();
            if (xSemaphoreTake(fb_mutex, portMAX_DELAY) && xSemaphoreTake(tcp_mutex, portMAX_DELAY))
            {
                tcp_send_dims(shared_fb);
                xSemaphoreGive(fb_mutex);
                xSemaphoreGive(tcp_mutex);
            }
            int64_t b = esp_timer_get_time();
            dims_accu += b - a;
            dims_count++;
        }

        // NEW FRAME
        if (event_val & EVENT_NEW_FRAME)
        {
            int64_t a = esp_timer_get_time();
            if (xSemaphoreTake(fb_mutex, portMAX_DELAY) && xSemaphoreTake(tcp_mutex, portMAX_DELAY))
            {
                tcp_send_frame(shared_fb);
                xSemaphoreGive(fb_mutex);
                xSemaphoreGive(tcp_mutex);
            }
            int64_t b = esp_timer_get_time();
            frame_accu += b - a;
            frame_count++;
        }

        // NEW BOX
        if (event_val & EVENT_NEW_BOX)
        {
            int64_t a = esp_timer_get_time();
            if (xSemaphoreTake(box_mutex, portMAX_DELAY) && xSemaphoreTake(tcp_mutex, portMAX_DELAY))
            {
                tcp_send_box(shared_box);
                xSemaphoreGive(box_mutex);
                xSemaphoreGive(tcp_mutex);
            }
            int64_t b = esp_timer_get_time();
            box_accu += b - a;
            box_count++;
        }

        int64_t loop_end = esp_timer_get_time();
        loop_total_accu += loop_end - loop_start;
        loop_count++;

        elapsed_time += (loop_end - loop_start) / 1e6f;

        if (elapsed_time >= 10.0f)
        {
            printf(
                "\n=== Image Send Task Averages (last %.1f s) ===\n"
                "dims send:   %llu us (%.3f ms) over %lu events\n"
                "frame send:  %llu us (%.3f ms) over %lu events\n"
                "box send:    %llu us (%.3f ms) over %lu events\n"
                "loop total:  %llu us (%.3f ms) over %lu loops\n\n",
                elapsed_time,
                dims_count ? (unsigned long long)(dims_accu / dims_count) : 0,
                dims_count ? (dims_accu / dims_count) / 1000.0 : 0.0, dims_count,
                frame_count ? (unsigned long long)(frame_accu / frame_count) : 0,
                frame_count ? (frame_accu / frame_count) / 1000.0 : 0.0, frame_count,
                box_count ? (unsigned long long)(box_accu / box_count) : 0,
                box_count ? (box_accu / box_count) / 1000.0 : 0.0, box_count,
                loop_count ? (unsigned long long)(loop_total_accu / loop_count) : 0,
                loop_count ? (loop_total_accu / loop_count) / 1000.0 : 0.0, loop_count);

            // reset accumulators
            dims_accu = frame_accu = box_accu = loop_total_accu = 0;
            dims_count = frame_count = box_count = loop_count = 0;
            elapsed_time = 0.0f;
        }
    }
}

void print_task_stats(void)
{
    char *buffer = malloc(1024); // Stupid malloc, do a static allocation once
    if (buffer == NULL)
    {
        return;
    }
    vTaskGetRunTimeStats(buffer);
    printf("\nTask Run-Time Stats:\n");
    printf("Name          Time (ticks)    Percentage\n");
    printf("-----------------------------------------\n");
    printf("%s\n", buffer);
    free(buffer);
}

void stats_task(void *arg)
{
    while (1)
    {
        print_task_stats();
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_wifi();

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        return;
    }

    fb_mutex = xSemaphoreCreateMutex();
    box_mutex = xSemaphoreCreateMutex();
    tcp_mutex = xSemaphoreCreateMutex();

    shared_fb = &shared_fb_instance;
    shared_fb->buf = shared_fb_buf;
    shared_box = &shared_box_instance;

    xTaskCreate(tcp_server_task, "TCP_SERVER_TASK", 8192, NULL, 4, &tcp_server_handle);
    xTaskCreatePinnedToCore(image_proc_task, "IMAGE_PROC_TASK", 16384, NULL, 6, &image_proc_handle, 1);
    xTaskCreatePinnedToCore(image_send_task, "IMAGE_SEND_TASK", 8192, NULL, 5, &image_send_handle, 0);
    xTaskCreate(stats_task, "TASK_STATS_TASK", 8192, NULL, 4, NULL);
}
