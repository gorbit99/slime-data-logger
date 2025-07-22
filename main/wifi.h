#pragma once

#include <esp_check.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <sys/socket.h>

void init_wifi(QueueHandle_t packetQueue);
void tcp_task(void *userArg);
void tcp_receive_task(void *userArg);
void wifi_event_handler(void *userArg,
                        esp_event_base_t eventBase,
                        int32_t eventId,
                        void *eventData);
