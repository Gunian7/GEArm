#include "esp8266_mqtt.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"

// 发送 AT 指令
void esp8266_send_at_command(const char *command)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)command, strlen(command), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
}

// 等待 AT 指令响应
int esp8266_wait_response(const char *response, uint32_t timeout)
{
    uint32_t start_time = HAL_GetTick();
    while (HAL_GetTick() - start_time < timeout)
    {   
        HAL_UART_Receive(&huart3, (uint8_t *)uart3_rx_buff, UART3_RX_BUFF_SIZE, 1000);
        // LOG("%s\n", uart3_rx_buff);
        if (strstr((char *)uart3_rx_buff, response) != NULL)
        {
            memset((char *)uart3_rx_buff, 0, UART3_RX_BUFF_SIZE);
            return 1;
        }
    }
    return 0;
}

// ESP8266 连接 WiFi
int esp8266_connect_wifi(const char *ssid, const char *password)
{
   char command[100];

    // 重启 ESP8266
    LOG("Restarting ESP8266...\n");
    esp8266_send_at_command("AT+RST");
    if (!esp8266_wait_response("OK", 10000)) {
        LOG("Failed to restart ESP8266!\n"); // 重启ESP8266失败
        return 0;
    }
    vTaskDelay(1000);

    // 设置为 STA 模式
    LOG("Setting to STA mode...\n");
    esp8266_send_at_command("AT+CWMODE=1");
    if (!esp8266_wait_response("OK", 10000)) {
        LOG("Failed to set to STA mode!\n"); // 设置为STA模式失败
        return 0;
    }
    vTaskDelay(1000);

    // 连接 WiFi
    LOG("Connecting to WiFi...\n");
    snprintf(command, sizeof(command), "AT+CWJAP=\"%s\",\"%s\"", ssid, password);
    esp8266_send_at_command(command);
    if (!esp8266_wait_response("OK", 10000)) {
        LOG("The WIFI account or password is incorrect!\n"); // WIFI账号密码错误
        return 0;
    }

    LOG("Success connected to WiFi: %s!\n", ssid);
    return 1;
}

// ESP8266 连接 MQTT 服务器
int esp8266_connect_mqtt(const char *server, uint16_t port, const char *client_id, const char *username, const char *password)
{
    char command[100];

    // 配置 MQTT 用户属性
    LOG("Configuring MQTT user attributes...\n");
    snprintf(command, sizeof(command), "AT+MQTTUSERCFG=0,1,\"%s\",\"%s\",\"%s\",0,0,\"\"", client_id, username, password);
    esp8266_send_at_command(command);
    if (!esp8266_wait_response("OK", 2000)) return 0;
    LOG("MQTT user attributes configured successfully!\n");

    // 连接 MQTT 服务器
    LOG("Connecting to MQTT server...\n");
    snprintf(command, sizeof(command), "AT+MQTTCONN=0,\"%s\",%d,0", server, port);
    esp8266_send_at_command(command);
    if (!esp8266_wait_response("OK", 5000)) return 0;
    LOG("Connected to MQTT server successfully!\n");

    return 1;
}

// ESP8266 订阅 MQTT 主题
int esp8266_subscribe_topic(const char *topic, uint8_t qos)
{
    char command[100];

    // 订阅主题
    LOG("Subscribing to topic: %s\n", topic);
    snprintf(command, sizeof(command), "AT+MQTTSUB=0,\"%s\",%d", topic, qos);
    esp8266_send_at_command(command);
    if (!esp8266_wait_response("OK", 2000)) return 0;
    LOG("Subscribed to topic successfully!\n");

    return 1;
}

// ESP8266 发布 MQTT 消息
int esp8266_publish_message(const char *topic, const char *message, uint8_t qos, uint32_t wait)
{
    char command[256];
    // 发布消息
    snprintf(command, sizeof(command), "AT+MQTTPUB=0,\"%s\",\"%s\",%d,0", topic, message, qos);
    esp8266_send_at_command(command);
    if (wait != 0) {
        if (!esp8266_wait_response("OK", wait)) {
            return 0;
        }
    }
    return 1;
}

int esp8266_mqtt_init(void) 
{
    // 连接 WiFi
    int ret = esp8266_connect_wifi(WIFI_SSID, WIFI_PASSWORD);
    if (ret == 0) {
        LOG("Failed to connect to WiFi!\n"); // 连接WiFi失败
        return 0; 
    }

    // 连接 MQTT 服务器
    ret = esp8266_connect_mqtt(MQTT_SERVER, MQTT_PORT, MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD);
    if (ret == 0) {
        LOG("Failed to connect to MQTT server!\n"); // 连接MQTT服务器失败
        return 0;
    }

    // 订阅主题
    ret = esp8266_subscribe_topic(MQTT_TOPIC, 0);
    if (ret == 0) {
        LOG("Failed to subscribe to topic!\n"); // 订阅主题失败
        return 0;
    }

    HAL_UART_Receive_IT(&huart3, (uint8_t *)&uart3_rx_buff[uart3_rx_pos], 1); // 使能接收中断

    LOG("ESP8266 MQTT service initialized successfully!\n");
    // // 发布消息
    // esp8266_publish_message(MQTT_TOPIC, "Hello MQTT!", 0);
    return 1;
}
