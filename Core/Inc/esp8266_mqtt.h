#ifndef ESP8266_MQTT_H
#define ESP8266_MQTT_H

#include "main.h"

// 定义 WiFi 信息
#define WIFI_SSID       "xxxxxx"
#define WIFI_PASSWORD   "xxxxxx"

// 定义 MQTT 信息
#define MQTT_SERVER     "xxx.xxx.xxx"       // 替换为实际的 MQTT 服务器地址
#define MQTT_PORT       123                 // 替换为实际的 MQTT 服务器端口        
#define MQTT_CLIENT_ID  "xxxxxx"            // 替换为实际的 MQTT 客户端 ID
#define MQTT_TOPIC      "xxxx"              // 替换为实际的 MQTT 主题
#define MQTT_USERNAME   "xxxx"              // 替换为实际的 MQTT 用户名
#define MQTT_PASSWORD   "xxxx"              // 替换为实际的 MQTT 密码

// 发送 AT 指令
void esp8266_send_at_command(const char *command);
// 等待 AT 指令响应
int esp8266_wait_response(const char *response, uint32_t timeout);
// ESP8266 连接 WiFi
int esp8266_connect_wifi(const char *ssid, const char *password);
// ESP8266 连接 MQTT 服务器
int esp8266_connect_mqtt(const char *server, uint16_t port, const char *client_id, const char *username, const char *password);
// ESP8266 订阅 MQTT 主题
int esp8266_subscribe_topic(const char *topic, uint8_t qos);
// ESP8266 发布 MQTT 消息
int esp8266_publish_message(const char *topic, const char *message, uint8_t qos, uint32_t wait);
// 初始化 ESP8266
int esp8266_mqtt_init(void);

#endif
