/* FreeRTOS includes */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

/* ESP-IDF includes */
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

/* Network transport */
#include "network_transport.h"

/* coreMQTT-Agent network manager */
#include "core_mqtt_agent_network_manager.h"

/* Wifi provisioning/connection handler */
#include "app_wifi.h"

static const char *TAG = "main";

static NetworkContext_t xNetworkContext;

/* TODO - Config to be moved to kconfig */
#define CONFIG_NETWORK_MANAGER_HOSTNAME "a2np9zbvnebvto-ats.iot.us-west-2.amazonaws.com"
#define CONFIG_NETWORK_MANAGER_PORT 8883

/* Network credentials */
extern const char pcServerRootCAPem[] asm("_binary_root_cert_auth_pem_start");
extern const char pcClientCertPem[] asm("_binary_client_crt_start");
extern const char pcClientKeyPem[] asm("_binary_client_key_start");

// extern void vStartLargeMessageSubscribePublishTask( configSTACK_DEPTH_TYPE uxStackSize,
//     UBaseType_t uxPriority );
extern void vStartOTACodeSigningDemo( configSTACK_DEPTH_TYPE uxStackSize,
                                      UBaseType_t uxPriority );

void app_main(void)
{
    /* Initialize network context */
    xNetworkContext.pcHostname = CONFIG_NETWORK_MANAGER_HOSTNAME;
    xNetworkContext.xPort = CONFIG_NETWORK_MANAGER_PORT;
    xNetworkContext.pcServerRootCAPem = pcServerRootCAPem;
    xNetworkContext.pcClientCertPem = pcClientCertPem;
    xNetworkContext.pcClientKeyPem = pcClientKeyPem;
    xNetworkContext.pxTls = NULL;
    xNetworkContext.xTlsContextSemaphore = xSemaphoreCreateMutex();

    /* Initialize NVS partition */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* NVS partition was truncated
         * and needs to be erased */
        ESP_ERROR_CHECK(nvs_flash_erase());

        /* Retry nvs_flash_init */
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    xCoreMqttAgentNetworkManagerStart(&xNetworkContext);

    /* Start wifi */
    app_wifi_init();
    app_wifi_start(POP_TYPE_MAC);

    vStartOTACodeSigningDemo(4096, 2);
    
}