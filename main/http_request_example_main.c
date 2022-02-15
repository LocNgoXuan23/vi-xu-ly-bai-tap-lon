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

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/dac.h"
#include "esp_system.h"
#include <math.h>

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

#define DAC_EXAMPLE_CHANNEL     0
#define ADC2_EXAMPLE_CHANNEL    7
#define Voltage_Resolution 3.3
#define RL 10
#define a 102.2
#define b -2.473


TimerHandle_t xTimers[ NUM_TIMERS ];

static const char *TAG = "example";

char REQUEST[512];
char SUBREQUEST[100];
char recv_buf[512];

int adcValue;
float voltage;
float rs;
float r0 = 0;
float ratio;
float PPM;

static const adc_bits_width_t width = ADC_WIDTH_BIT_12;

int getADC() {
    uint8_t output_data=0;
    int     read_raw;
    esp_err_t r;

    gpio_num_t adc_gpio_num, dac_gpio_num;

    r = adc2_pad_get_io_num( ADC2_EXAMPLE_CHANNEL, &adc_gpio_num );
    assert( r == ESP_OK );
    r = dac_pad_get_io_num( DAC_EXAMPLE_CHANNEL, &dac_gpio_num );
    assert( r == ESP_OK );

    dac_output_enable( DAC_EXAMPLE_CHANNEL );

    //be sure to do the init before using adc2. 
    adc2_config_channel_atten( ADC2_EXAMPLE_CHANNEL, ADC_ATTEN_11db );


    dac_output_voltage( DAC_EXAMPLE_CHANNEL, output_data++ );
    r = adc2_get_raw( ADC2_EXAMPLE_CHANNEL, width, &read_raw);
    if ( r == ESP_OK ) {
        
    } else if ( r == ESP_ERR_INVALID_STATE ) {
        printf("%s: ADC2 not initialized yet.\n", esp_err_to_name(r));
    } else if ( r == ESP_ERR_TIMEOUT ) {
        //This can not happen in this example. But if WiFi is in use, such error code could be returned.
        printf("%s: ADC2 is in use by Wi-Fi.\n", esp_err_to_name(r));
    } else {
        printf("%s\n", esp_err_to_name(r));
    }

    return read_raw;
}

float getVoltage(int adcValue) {
    float voltage;
    voltage = (adcValue) * Voltage_Resolution / 4095;
    // printf("voltage = %f\n", voltage );
    return voltage;
}

float getRS(float voltage) {
    float rs = ((Voltage_Resolution * RL)/voltage) - RL;
    // printf("rs = %f\n", rs);
    return rs;
}

float getR0(float voltage) {
    float r0 = ((Voltage_Resolution * RL)/voltage) - RL;
    r0 = r0 / 3.6;
    // printf("r0 = %f\n", r0 );
    return r0;
}

float getRatio(float rs, float r0) {
    float ratio = rs / r0;
    // printf("ratio = %f\n", ratio );
    return ratio;
}

float getPPM(float r0) {
    int adcValue;
    float voltage;
    float rs;
    float ratio;
    float PPM;

    adcValue =  getADC();
    voltage = getVoltage(adcValue);
    rs = getRS(voltage);
    ratio = getRatio(rs, r0);
    PPM = a*pow(ratio, b);
    return PPM;
}

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
            return;
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
            return;
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
        float PPM;
		
		int status = errorHandler(ret);
        if (status == 0) return;

        // PPM = getPPM(r0);
        // PPM = r0;

        
        printf("Temperature is %.1f \n", getTemperature());
        printf("Humidity is %.1f\n", getHumidity());
        // printf("PPM = %f", PPM );


        // sprintf(SUBREQUEST, "api_key=FLWOURMSDTF3LKGU&temperature=%d&humidity=%d&co2=%d", 200, 300, 100);
        // sprintf(REQUEST, "POST /update.json HTTP/1.1\nHost: api.thingspeak.com\nConnection: close\nContent-Type: application/x-www-form-urlencoded\nContent-Length:%d\n\n%s\n", strlen(SUBREQUEST), SUBREQUEST);
        sprintf(REQUEST, "GET http://api.thingspeak.com/update?api_key=Y4XKGG93U5UUZDEH&field1=%f&field2=%f&field3=%f\n\n", getTemperature(), getHumidity(), getHumidity());
        // sprintf(REQUEST, "GET http://api.thingspeak.com/channels/1638336/feeds.json?api_key=5QGPD6BOMURAERFK&results=2\n\n");

        if (write(s, REQUEST, strlen(REQUEST)) < 0) {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "3333333333333333333333333333333333333333333333333333333333333333333333333333333333");
            return;
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
            return;
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
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Starting again!");
    }
    else if (ulCount == 1)
    {
        printf("Hello!\n");
        // float PPM = getADC();
        // printf("PPMxxxxxxxxxx = %f\n", PPM );
        // printf("\n-----------\n" );
    }
}

void postSever(float ppm, float temperature, float humidity) {
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
        return;
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
        return;
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

    // int ret = readDHT();
    
    // int status = errorHandler(ret);
    // if (status == 0) return;


    printf("================================post==================================\n" );
    printf("Temperature is %.1f \n", temperature);
    printf("Humidity is %.1f\n", humidity);
    printf("PPM = %f\n", ppm );
    printf("======================================================================" );

    // sprintf(SUBREQUEST, "api_key=FLWOURMSDTF3LKGU&temperature=%d&humidity=%d&co2=%d", 200, 300, 100);
    // sprintf(REQUEST, "POST /update.json HTTP/1.1\nHost: api.thingspeak.com\nConnection: close\nContent-Type: application/x-www-form-urlencoded\nContent-Length:%d\n\n%s\n", strlen(SUBREQUEST), SUBREQUEST);
    sprintf(REQUEST, "GET http://api.thingspeak.com/update?api_key=UM427T7864Q6ORA5&field1=%f&field2=%f&field3=%f\n\n", temperature, humidity, ppm);
    // sprintf(REQUEST, "GET http://api.thingspeak.com/channels/1638336/feeds.json?api_key=5QGPD6BOMURAERFK&results=2\n\n");

    if (write(s, REQUEST, strlen(REQUEST)) < 0) {
        ESP_LOGE(TAG, "... socket send failed");
        close(s);
        vTaskDelay(4000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "3333333333333333333333333333333333333333333333333333333333333333333333333333333333");
        return;
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
        return;
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


void app_main()
{
    setDHTgpio( 4 );
    
    printf("\n----------------------------------------Reset--------------------------------------------------\n" );
    // for(int i = 1; i<=10; i ++) {
    //     adcValue =  getADC();
    //     voltage = getVoltage(adcValue);
    //     r0 += getR0(voltage);
    // }
    // r0 = r0 / 10;
    // printf("r0 = %f", r0 );
    // float PPM = getPPM(r0);
    // printf("| PPM = %f", PPM );
    // printf("\n------End------\n" );

    // ESP_ERROR_CHECK( nvs_flash_init() );
    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());

    // xTimers[0] = xTimerCreate("TimerBlink", pdMS_TO_TICKS(5000), pdTRUE, ( void * ) 0, vTimerCallback);
    // xTimers[1] = xTimerCreate("TimerBlink", pdMS_TO_TICKS(1000), pdTRUE, ( void * ) 1, vTimerCallback);

    // xTimerStart(xTimers[0], 0);
    // xTimerStart(xTimers[1], 0);

    // ESP_ERROR_CHECK(example_connect());

    

    while(1) {
        int ret = readDHT();
		
		int status = errorHandler(ret);
        if (status == 0) continue;

        float temperature = getTemperature();
        float humidity = getHumidity();
        
        printf("Temperature is %.5f \n", temperature);
        printf("Humidity is %.5f\n", humidity);

        PPM = getPPM(2);
        printf("PPM = %f", PPM );
        printf("| ADC = %d\n", getADC() );
        
        printf("\n-----------\n" );
        vTaskDelay(500 / portTICK_PERIOD_MS);


        ESP_ERROR_CHECK( nvs_flash_init() );
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        ESP_ERROR_CHECK(example_connect());
        postSever(PPM, temperature, humidity);

        esp_wifi_disconnect();
        vTaskDelay(600 * 1000 / portTICK_PERIOD_MS);
    }


  
}