#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "led_strip.h"
// OTA
#include "esp_http_client.h"
#include "esp_ota_ops.h"
#include "esp_https_ota.h"


/** DEFINES **/
#define WIFI_SUCCESS 1 << 0
#define WIFI_FAILURE 1 << 1
#define TCP_SUCCESS 1 << 0
#define TCP_FAILURE 1 << 1
#define MAX_FAILURES 10

#define LED_STRIP_BLINK_GPIO  48
#define LED_STRIP_LED_NUMBERS 1
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

#define FIRMWARE_VERSION 1
#define OTA_URL "http://192.168.0.106/jeeves/esp_led_ota/esp32-led.bin"
#define OTA_TASK_STACK_SIZE 8192


/** GLOBALS **/

// event group to contain status information
static EventGroupHandle_t wifi_event_group;

// retry tracker
static int s_retry_num = 0;

// task tag
static const char *TAG = "WIFI";

// global

/** FUNCTIONS **/
led_strip_handle_t configure_led(void)
{
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_BLINK_GPIO,   // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_NUMBERS,        // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812,            // LED strip model
        .flags.invert_out = false,                // whether to invert the output signal
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
        .rmt_channel = 0,
#else
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .flags.with_dma = false,               // DMA feature is available on ESP target like ESP32-S3
#endif
    };

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    return led_strip;
}


//event handler for wifi events
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
	{
		ESP_LOGI(TAG, "Connecting to AP...");
		esp_wifi_connect();
	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
	{
        wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
        ESP_LOGI(TAG, "Disconnected from AP, reason: %d", event->reason);
		if (s_retry_num < MAX_FAILURES)
		{
			ESP_LOGI(TAG, "Reconnecting to AP...");
            vTaskDelay(1000 / portTICK_PERIOD_MS); 
			esp_wifi_connect();
			s_retry_num++;
		} else {
			xEventGroupSetBits(wifi_event_group, WIFI_FAILURE);
		}
	}
}

//event handler for ip events
static void ip_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
	if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
	{
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "STA IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_SUCCESS);
    }

}

esp_err_t check_for_updates(void) {
    esp_http_client_config_t config = {
        .url = OTA_URL,
        .cert_pem = NULL,
    };

    esp_err_t ret = esp_https_ota(&config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "OTA update successful, restarting...");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA update failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

int get_latest_version(void) {
    // For simplicity, assume we get the latest version number from a predefined URL
    // In practice, implement the HTTP GET request to fetch the version
    return 2; // Example: pretend we fetched version 2 from the server
}

void log_firmware_version(void) {
    ESP_LOGI(TAG, "Current firmware version: %d", FIRMWARE_VERSION);
}


void ota_task(void *pvParameter) {
    int latest_version = get_latest_version();
    if (latest_version > FIRMWARE_VERSION) {
        ESP_LOGI(TAG, "New firmware version available: %d", latest_version);
        check_for_updates();
        vTaskDelete(NULL);
    } else {
        ESP_LOGI(TAG, "No new firmware version available");
    }
}

// connect to wifi and return the result
esp_err_t connect_wifi()
{
	int status = WIFI_FAILURE;

	/** INITIALIZE ALL THE THINGS **/
	//initialize the esp network interface
	ESP_ERROR_CHECK(esp_netif_init());

	//initialize default esp event loop
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	//create wifi station in the wifi driver
	esp_netif_create_default_wifi_sta();

	//setup wifi station with the default wifi configuration
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /** EVENT LOOP CRAZINESS **/
	wifi_event_group = xEventGroupCreate();

    esp_event_handler_instance_t wifi_handler_event_instance;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &wifi_handler_event_instance));

    esp_event_handler_instance_t got_ip_event_instance;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &ip_event_handler,
                                                        NULL,
                                                        &got_ip_event_instance));

    /** START THE WIFI DRIVER **/
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "3210",
            .password = "moremore",
        .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    // set the wifi controller to be a station
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );

    // set the wifi config
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );

    // start the wifi driver
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "STA initialization complete");

    /** NOW WE WAIT **/
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
            WIFI_SUCCESS | WIFI_FAILURE,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_SUCCESS) {
        ESP_LOGI(TAG, "Connected to ap");
        status = WIFI_SUCCESS;
    } else if (bits & WIFI_FAILURE) {
        ESP_LOGI(TAG, "Failed to connect to ap");
        status = WIFI_FAILURE;
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
        status = WIFI_FAILURE;
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, got_ip_event_instance));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_handler_event_instance));
    vEventGroupDelete(wifi_event_group);

    return status;
}

// connect to the server and return the result
void handle_led_command(led_strip_handle_t led_strip, const char* cmd) {
    if (strcmp(cmd, "ON") == 0 || strcmp(cmd, "1") == 0) {
        for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 5, 5, 5));
        }
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));
        ESP_LOGI(TAG, "LED turned ON");
    } else if (strcmp(cmd, "OFF") == 0 || strcmp(cmd, "0") == 0) {
        ESP_ERROR_CHECK(led_strip_clear(led_strip));
        ESP_LOGI(TAG, "LED turned OFF");
    } else if (strcmp(cmd, "OTA") == 0) {
        ESP_LOGI(TAG, "Attempting checking for updates...");
        xTaskCreate(&ota_task, "ota_task", OTA_TASK_STACK_SIZE, NULL, 5, NULL);
    } else {
        ESP_LOGI(TAG, "Unknown command: %s", cmd);
    }
}

esp_err_t connect_tcp_server(led_strip_handle_t led_strip) {
    struct sockaddr_in serverInfo = {0};
    char readBuffer[1024] = {0};

    serverInfo.sin_family = AF_INET;
    serverInfo.sin_addr.s_addr = inet_addr("192.168.0.106");
    serverInfo.sin_port = htons(12345);


    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create a socket..?");
        return TCP_FAILURE;
    }

    if (connect(sock, (struct sockaddr *)&serverInfo, sizeof(serverInfo)) != 0) {
        ESP_LOGE(TAG, "Failed to connect to %s!", inet_ntoa(serverInfo.sin_addr.s_addr));
        close(sock);
        return TCP_FAILURE;
    }

    ESP_LOGI(TAG, "Connected to TCP server.");
    while (1) {
        int r = read(sock, readBuffer, sizeof(readBuffer) - 1);
        if (r > 0) {
            readBuffer[r] = 0; // Null-terminate the received data
            ESP_LOGI(TAG, "Received: %s", readBuffer);
            handle_led_command(led_strip, readBuffer);
        } else if (r == 0) {
            ESP_LOGI(TAG, "Connection closed by server");
            break;
        } else {
            ESP_LOGE(TAG, "Error reading from socket");
            break;
        }
    }

    close(sock);
    return TCP_SUCCESS;
}

void app_main(void)
{
	esp_err_t status = WIFI_FAILURE;

	//initialize storage
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    log_firmware_version();
    //initialize LED
    led_strip_handle_t led_strip = configure_led();


    // connect to wireless AP
	status = connect_wifi();
	if (WIFI_SUCCESS != status)
	{
		ESP_LOGI(TAG, "Failed to associate to AP, dying...");
		return;
	}
    log_firmware_version();
    ESP_LOGI(TAG, "Connected to AP, checking for OTA updates...");

    // Check and update firmware
    // xTaskCreate(&ota_task, "ota_task", OTA_TASK_STACK_SIZE, NULL, 5, NULL);
	
    ESP_LOGI(TAG, "Connected to AP, starting TCP connection...");
    
	status = connect_tcp_server(led_strip);
	if (TCP_SUCCESS != status)
	{
		ESP_LOGI(TAG, "Failed to connect to remote server, dying...");
		return;
	}
    
}