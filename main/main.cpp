#include "freertos/idf_additions.h"
#include "imu.h"
#include "packets.h"
#include "wifi.h"

static const char *TAG = "MAIN";

extern "C" void app_main() {
    QueueHandle_t packetQueue = xQueueCreate(16, sizeof(Packet));
    init_wifi(packetQueue);
    init_imu(packetQueue);
}
