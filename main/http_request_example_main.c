/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
// #include "input_iot.h"
// #include "output_iot.h"
// #include "dht11.h"
// #include "DHT22.h"

#include <string.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "DHT22.h"

/* Constants that aren't configurable in menuconfig */
#define WEB_SERVER "api.thingspeak.com"
#define WEB_PORT "80"
#define WEB_PATH "/"

#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

#define NUM_TIMERS 2

TimerHandle_t xTimers[ NUM_TIMERS ];

static const char *TAG = "example";

char REQUEST[512];
char SUBREQUEST[100];
char recv_buf[512];



void vTimerCallback( TimerHandle_t xTimer )
{
    configASSERT( xTimer );
    int ulCount = ( uint32_t ) pvTimerGetTimerID( xTimer );
    if (ulCount == 0)
    {
        const struct addrinfo hints = {
            .ai_family = AF_INET,
            .ai_socktype = SOCK_STREAM,
        };
        struct addrinfo *res;
        struct in_addr *addr;
        int s, r;

        int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);

        if(err != 0 || res == NULL) {
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "1111111111111111111111111111111111111111111111111111111111111111111111111111111111");

        }

        /* Code to print the resolved IP.

        Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
        addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
        ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

        s = socket(res->ai_family, res->ai_socktype, 0);
        if(s < 0) {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "2222222222222222222222222222222222222222222222222222222222222222222222222222222222");
        }
        ESP_LOGI(TAG, "... allocated socket");


        if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
        }

        ESP_LOGI(TAG, "... connected");
        freeaddrinfo(res);

        int ret = readDHT();
		
		errorHandler(ret);

        
        // printf("Temperature is %.1f \n", getTemperature());
        // printf("Humidity is %.1f\n", getHumidity());

        // sprintf(SUBREQUEST, "api_key=FLWOURMSDTF3LKGU&temperature=%d&humidity=%d&co2=%d", 200, 300, 100);
        // sprintf(REQUEST, "POST /update.json HTTP/1.1\nHost: api.thingspeak.com\nConnection: close\nContent-Type: application/x-www-form-urlencoded\nContent-Length:%d\n\n%s\n", strlen(SUBREQUEST), SUBREQUEST);
        sprintf(REQUEST, "GET http://api.thingspeak.com/update?api_key=FLWOURMSDTF3LKGU&field1=%f&field2=%f&field3=%f\n\n", getTemperature(), getHumidity(), getHumidity());
        // sprintf(REQUEST, "GET http://api.thingspeak.com/channels/1638336/feeds.json?api_key=5QGPD6BOMURAERFK&results=2\n\n");

        if (write(s, REQUEST, strlen(REQUEST)) < 0) {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "3333333333333333333333333333333333333333333333333333333333333333333333333333333333");

        }
        ESP_LOGI(TAG, "... socket send success");


        struct timeval receiving_timeout;
        receiving_timeout.tv_sec = 5;
        receiving_timeout.tv_usec = 0;
        if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                sizeof(receiving_timeout)) < 0) {
            ESP_LOGE(TAG, "... failed to set socket receiving timeout");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "4444444444444444444444444444444444444444444444444444444444444444444444444444444444");
        }
        ESP_LOGI(TAG, "... set socket receiving timeout success");

        /* Read HTTP response */
        do {
            bzero(recv_buf, sizeof(recv_buf));
            r = read(s, recv_buf, sizeof(recv_buf)-1);
            for(int i = 0; i < r; i++) {
                putchar(recv_buf[i]);
            }
        } while(r > 0);

        ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d.", r, errno);
        close(s);
        // for(int countdown = 10; countdown >= 0; countdown--) {
        //     ESP_LOGI(TAG, "%d... ", countdown);
        //     vTaskDelay(1000 / portTICK_PERIOD_MS);
        //     ESP_LOGI(TAG, "5555555555555555555555555555555555555555555555555555555555555555555555555555555555");
        // }
        ESP_LOGI(TAG, "Starting again!");
    }
    else if (ulCount == 1)
    {
        printf("Hello!\n");
    }
}

static void http_get_task(void *pvParameters)
{
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;

    while(1) {

        int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);

        if(err != 0 || res == NULL) {
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "1111111111111111111111111111111111111111111111111111111111111111111111111111111111");

            continue;
        }

        /* Code to print the resolved IP.

           Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
        addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
        ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

        s = socket(res->ai_family, res->ai_socktype, 0);
        if(s < 0) {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "2222222222222222222222222222222222222222222222222222222222222222222222222222222222");
            continue;
        }
        ESP_LOGI(TAG, "... allocated socket");


        if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "... connected");
        freeaddrinfo(res);


        sprintf(SUBREQUEST, "api_key=ZTG7CI12UJRM7DGJ&field1=%d&field2=%d", 200, 200);
        sprintf(REQUEST, "POST /update.json HTTP/1.1\nHost: api.thingspeak.com\nConnection: close\nContent-Type: application/x-www-form-urlencoded\nContent-Length:%d\n\n%s\n", strlen(SUBREQUEST), SUBREQUEST);
        // sprintf(REQUEST, "GET http://api.thingspeak.com/update.json?api_key=ZTG7CI12UJRM7DGJ&field1=80&field2=80\n\n");
        // sprintf(REQUEST, "GET http://api.thingspeak.com/channels/1638336/feeds.json?api_key=5QGPD6BOMURAERFK&results=2\n\n");

        if (write(s, REQUEST, strlen(REQUEST)) < 0) {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "3333333333333333333333333333333333333333333333333333333333333333333333333333333333");

            continue;
        }
        ESP_LOGI(TAG, "... socket send success");


        struct timeval receiving_timeout;
        receiving_timeout.tv_sec = 5;
        receiving_timeout.tv_usec = 0;
        if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                sizeof(receiving_timeout)) < 0) {
            ESP_LOGE(TAG, "... failed to set socket receiving timeout");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "4444444444444444444444444444444444444444444444444444444444444444444444444444444444");
            continue;
        }
        ESP_LOGI(TAG, "... set socket receiving timeout success");

        /* Read HTTP response */
        do {
            bzero(recv_buf, sizeof(recv_buf));
            r = read(s, recv_buf, sizeof(recv_buf)-1);
            for(int i = 0; i < r; i++) {
                putchar(recv_buf[i]);
            }
        } while(r > 0);

        ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d.", r, errno);
        close(s);
        // for(int countdown = 10; countdown >= 0; countdown--) {
        //     ESP_LOGI(TAG, "%d... ", countdown);
        //     vTaskDelay(1000 / portTICK_PERIOD_MS);
        //     ESP_LOGI(TAG, "5555555555555555555555555555555555555555555555555555555555555555555555555555555555");
        // }
        ESP_LOGI(TAG, "Starting again!");
    }
}



void app_main()
{
    setDHTgpio( 4 );

    ESP_ERROR_CHECK( nvs_flash_init() );
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    xTimers[0] = xTimerCreate("TimerBlink", pdMS_TO_TICKS(5000), pdTRUE, ( void * ) 0, vTimerCallback);
    xTimers[1] = xTimerCreate("TimerBlink", pdMS_TO_TICKS(1000), pdTRUE, ( void * ) 1, vTimerCallback);

    xTimerStart(xTimers[0], 0);
    // xTimerStart(xTimers[1], 0);

    ESP_ERROR_CHECK(example_connect());
    // xTaskCreate(&http_get_task, "http_get_task", 4096, NULL, 5, NULL);

    
}