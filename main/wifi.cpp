#include "wifi.h"

#include "freertos/idf_additions.h"
#include "packets.h"

static const char *TAG = "WIFI";

static constexpr auto WIFI_CONNECTED_BIT = BIT(0);
static constexpr auto WIFI_FAIL_BIT = BIT(1);

bool connected = false;

void init_wifi(QueueHandle_t packetQueue) {
    auto result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES
        || result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        result = nvs_flash_init();
    }
    ESP_ERROR_CHECK(result);

    auto eventGroup = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifiInitConfig = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifiInitConfig));

    esp_event_handler_instance_t wifiAnyHandler;
    esp_event_handler_instance_t gotIpHandler;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        &eventGroup,
                                                        &wifiAnyHandler));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        &eventGroup,
                                                        &gotIpHandler));

    wifi_config_t wifiConfig = {
            .sta = {
                    .ssid = CONFIG_WIFI_SSID,
                    .password = CONFIG_WIFI_PASS,
                    .threshold =
                            {
                                    .authmode = WIFI_AUTH_WPA2_PSK,
                            },
            }};

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifiConfig));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi initialized");

    auto eventBits = xEventGroupWaitBits(eventGroup,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         false,
                                         false,
                                         portMAX_DELAY);

    if (eventBits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to Wi-Fi!");
    } else if (eventBits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to Wi-Fi!");
    }

    xTaskCreate(tcp_task, "tcp", 4096, &packetQueue, 5, nullptr);
}

void wifi_event_handler(void *userArg,
                        esp_event_base_t eventBase,
                        int32_t eventId,
                        void *eventData) {
    static int retryNum = 0;
    auto eventGroup = *reinterpret_cast<EventGroupHandle_t *>(userArg);

    if (eventBase == WIFI_EVENT && eventId == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (eventBase == WIFI_EVENT
               && eventId == WIFI_EVENT_STA_DISCONNECTED) {
        if (retryNum < 3) {
            esp_wifi_connect();
            retryNum++;
            ESP_LOGI(TAG, "Retrying connection to the Wi-Fi AP");
        } else {
            xEventGroupSetBits(eventGroup, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "Failed to connect to the Wi-Fi AP");
    } else if (eventBase == IP_EVENT && eventId == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event =
                reinterpret_cast<ip_event_got_ip_t *>(eventData);
        ESP_LOGI(TAG, "Got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        retryNum = 0;
        xEventGroupSetBits(eventGroup, WIFI_CONNECTED_BIT);
    }
}

void tcp_task(void *userArg) {
    char hostIp[] = CONFIG_SERVER_IP;
    QueueHandle_t packetQueue = *reinterpret_cast<QueueHandle_t *>(userArg);

    while (true) {
        sockaddr_in destAddress;
        inet_pton(AF_INET, hostIp, &destAddress.sin_addr);
        destAddress.sin_family = AF_INET;
        destAddress.sin_port = htons(CONFIG_SERVER_PORT);

        int socketHandle = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (socketHandle < 0) {
            ESP_LOGE(TAG,
                     "Couldn't create TCP socket, reason: %s",
                     strerror(errno));
            abort();
        }
        ESP_LOGI(TAG, "Socket created, connecting...");

        int error = connect(socketHandle,
                            reinterpret_cast<sockaddr *>(&destAddress),
                            sizeof(destAddress));
        if (error != 0) {
            ESP_LOGE(TAG, "Unable to connect, reason: %s", strerror(errno));
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "Retrying");
            close(socketHandle);
            continue;
        }
        ESP_LOGI(TAG, "Connected successfully!");

        TaskHandle_t receiveTaskHandle;
        xTaskCreate(tcp_receive_task,
                    "tcp_receive",
                    4096,
                    &socketHandle,
                    5,
                    &receiveTaskHandle);

        bool wasConnected = false;

        while (true) {
            if (!connected) {
                uint8_t payload[] = {
                        static_cast<uint8_t>(SendCodes::Ping),
                };

                ESP_LOGI(TAG, "Ping...");

                int result = send(socketHandle, payload, sizeof(payload), 0);
                if (result < 0) {
                    ESP_LOGE(TAG,
                             "Error while sending ping request, reason: %s",
                             strerror(errno));
                    break;
                }

                vTaskDelay(1000 / portTICK_PERIOD_MS);
                continue;
            }

            if (connected && !wasConnected) {
                wasConnected = true;
                xQueueReset(packetQueue);
            }

            Packet packet;
            while (xQueueReceive(packetQueue, &packet, portMAX_DELAY)) {
                int result = send(socketHandle, &packet, sizeof(packet), 0);
                if (result < 0) {
                    ESP_LOGE(TAG,
                             "Error while sending packet, reason: %s",
                             strerror(errno));
                    break;
                }
            }
        }

        if (socketHandle != -1) {
            shutdown(socketHandle, 0);
            close(socketHandle);
        }

        vTaskDelete(receiveTaskHandle);

        ESP_LOGI(TAG, "Reconnecting...");
    }
}

void tcp_receive_task(void *userArg) {
    char rxBuffer[128];
    int socketHandle = *reinterpret_cast<int *>(userArg);

    while (true) {
        int len = recv(socketHandle, rxBuffer, sizeof(rxBuffer), 0);

        if (len < 0) {
            ESP_LOGE(TAG, "Error while receiving, reason: %s", strerror(errno));
            continue;
        }

        if (len == 0) {
            continue;
        }

        ReceiveCodes code = static_cast<ReceiveCodes>(rxBuffer[0]);

        switch (code) {
        case ReceiveCodes::Pong:
            connected = true;
            ESP_LOGI(TAG, "Found the server!");
            break;
        default:
            ESP_LOGE(TAG, "Unknown code received: %d", rxBuffer[0]);
            break;
        }
    }
}
