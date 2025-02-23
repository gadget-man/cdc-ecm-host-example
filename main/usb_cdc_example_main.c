/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"

#include "esp_event.h"
#include "esp_eth.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "lwip/init.h"
#include "lwip/ip_addr.h"
#include "lwip/dhcp.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"

#define EXAMPLE_USB_HOST_PRIORITY (20)
#define EXAMPLE_USB_DEVICE_VID (0x0BDA)
#define EXAMPLE_USB_DEVICE_PID (0x8152)
#define EXAMPLE_TX_STRING "Hello, World!"
#define EXAMPLE_TX_TIMEOUT_MS (1000)

static const char *TAG = "USB-CDC";
static SemaphoreHandle_t device_disconnected_sem;

cdc_acm_dev_hdl_t cdc_dev = NULL;
esp_netif_t *usb_netif = NULL;

/**
 * @brief Data received callback
 *
 * @param[in] data     Pointer to received data
 * @param[in] data_len Length of received data in bytes
 * @param[in] arg      Argument we passed to the device open function
 * @return
 *   true:  We have processed the received data
 *   false: We expect more data
 */
static bool handle_rx(const uint8_t *data, size_t data_len, void *arg)
{
    ESP_LOGI(TAG, "Data received");
    ESP_LOG_BUFFER_HEXDUMP(TAG, data, data_len, ESP_LOG_INFO);
    return true;
}

/**
 * @brief Device event callback
 *
 * Apart from handling device disconnection it doesn't do anything useful
 *
 * @param[in] event    Device event type and data
 * @param[in] user_ctx Argument we passed to the device open function
 */
static void handle_event(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    switch (event->type)
    {
    case CDC_ACM_HOST_ERROR:
        ESP_LOGE(TAG, "CDC-ACM error has occurred, err_no = %i", event->data.error);
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGI(TAG, "Device suddenly disconnected");
        ESP_ERROR_CHECK(cdc_acm_host_close(event->data.cdc_hdl));
        xSemaphoreGive(device_disconnected_sem);
        break;
    case CDC_ACM_HOST_SERIAL_STATE:
        ESP_LOGI(TAG, "Serial state notif 0x%04X", event->data.serial_state.val);
        break;
    case CDC_ACM_HOST_NETWORK_CONNECTION:
    default:
        ESP_LOGW(TAG, "Unsupported CDC event: %i", event->type);
        break;
    }
}

/** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    if (event_base == ETH_EVENT)
    {
        switch (event_id)
        {
        case ETHERNET_EVENT_CONNECTED:
            esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
            ESP_LOGI(TAG, "Ethernet Link Up");
            ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                     mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
            break;
        case ETHERNET_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "Ethernet Link Down");
            break;
        case ETHERNET_EVENT_START:
            ESP_LOGI(TAG, "Ethernet Started");
            break;
        case ETHERNET_EVENT_STOP:
            ESP_LOGI(TAG, "Ethernet Stopped");
            break;
        default:
            ESP_LOGI(TAG, "Unhandled Ethernet Event: %ld", event_id);
            break;
        }
    }
    else if (event_base == IP_EVENT)
    {
        switch (event_id)
        {
        case IP_EVENT_ETH_GOT_IP:
        {
            ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
            ESP_LOGI(TAG, "Ethernet got IP: " IPSTR, IP2STR(&event->ip_info.ip));
            ESP_LOGI(TAG, "Subnet Mask: " IPSTR, IP2STR(&event->ip_info.netmask));
            ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&event->ip_info.gw));
            break;
        }

        case IP_EVENT_ETH_LOST_IP:
            ESP_LOGI(TAG, "Ethernet lost IP address");
            break;

        default:
            ESP_LOGW(TAG, "Unhandled IP Event: %ld", event_id);
            break;
        }
    }
}

/**
 * @brief USB Host library handling task
 *
 * @param arg Unused
 */
static void usb_lib_task(void *arg)
{
    while (1)
    {
        // Start handling system events
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
        {
            ESP_ERROR_CHECK(usb_host_device_free_all());
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE)
        {
            ESP_LOGI(TAG, "USB: All devices freed");
            // Continue handling USB events to allow device reconnection
        }
    }
}

/**
 * @brief Callback from usb_ncm_init usb driver config for sending data over usb.
 *
 */
static esp_err_t netif_transmit(void *h, void *buffer, size_t len)
{
    ESP_LOGI(TAG, "Called netif_transmit with length %d", len);

    cdc_acm_dev_hdl_t cdc_dev = (cdc_acm_dev_hdl_t)h;

    // if (cdc_dev == NULL)
    // {
    //     ESP_LOGE(TAG, "CDC device handle is NULL!");
    //     return ESP_FAIL;
    // }
    // else
    // {
    //     ESP_LOGI(TAG, "CDC device handle is %p", cdc_dev);
    // }

    // ESP_LOGI(TAG, "Buffer: %s", (char *)buffer);
    if (cdc_acm_host_data_tx_blocking(cdc_dev, (const uint8_t *)buffer, len, 2000) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send buffer to USB!");
    }
    return ESP_OK;
}

static void l2_free(void *h, void *buffer)
{
    free(buffer);
}

/**
 * @brief Netif Initialisation
 *
 */
esp_err_t usb_ncm_init(cdc_acm_dev_hdl_t cdc_dev)
{
    ESP_LOGI(TAG, "Calling usb_ncm_init with handle %p", cdc_dev);
    esp_netif_init();
    esp_event_loop_create_default();

    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 0, 200);
    IP4_ADDR(&ip_info.gw, 192, 168, 0, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);

    //     // esp_netif_dhcpc_stop(usb_netif);
    // esp_netif_set_ip_info(usb_netif, &ip_info);
    //     // ESP_LOGI(TAG, "Static IP set: 192.168.0.220");
    //     // esp_netif_action_connected(usb_netif, NULL, 0, NULL);

    // 1) Derive the base config (very similar to IDF's default WiFi AP with DHCP server)
    esp_netif_inherent_config_t base_cfg = {
        .flags = ESP_NETIF_FLAG_EVENT_IP_MODIFIED | ESP_NETIF_FLAG_AUTOUP | ESP_NETIF_DHCP_CLIENT,
        .ip_info = &ip_info,
        .get_ip_event = IP_EVENT_ETH_GOT_IP,
        .lost_ip_event = IP_EVENT_ETH_LOST_IP,
        .if_key = "usb_eth",
        .if_desc = "usb ncm config device",
        .route_prio = 10};

    // 2) Use static config for driver's config pointing only to static transmit and free functions
    esp_netif_driver_ifconfig_t driver_cfg = {
        .handle = cdc_dev,               // not using an instance, USB-NCM is a static singleton (must be != NULL)
        .transmit = netif_transmit,      // point to static Tx function
        .driver_free_rx_buffer = l2_free // point to Free Rx buffer function
    };

    ESP_LOGI(TAG, "checking driver handle %p", driver_cfg.handle);

    // Config the esp-netif with:
    //   1) inherent config (behavioural settings of an interface)
    //   2) driver's config (connection to IO functions -- usb)
    //   3) stack config (using lwip IO functions -- derive from eth)
    esp_netif_config_t cfg = {
        .base = &base_cfg,
        .driver = &driver_cfg,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH, // USB-NCM is an Ethernet netif from lwip perspective, we already have IO definitions for that:
    };

    // Get factory MAC and set it
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_ETH);
    ESP_LOGI(TAG, "Set MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    esp_netif_set_mac(usb_netif, mac);
    // memcpy(usb_driver->mac_addr, mac, 6);

    esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, eth_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, eth_event_handler, NULL);

    usb_netif = esp_netif_new(&cfg);
    if (usb_netif == NULL)
    {
        return ESP_FAIL;
    }
    // esp_netif_set_mac(s_netif, mac);

    // 4) Start DHCP client to obtain IP address dynamically
    esp_err_t err = esp_netif_dhcpc_start(usb_netif);
    if (err == ESP_OK || err == ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED)
    {
        ESP_LOGI(TAG, "DHCP client started successfully");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to start DHCP client: %s", esp_err_to_name(err));
    }

    // start the interface manually (as the driver has been started already)
    esp_netif_action_start(usb_netif, 0, 0, 0);
    esp_netif_action_connected(usb_netif, NULL, 0, NULL);

    return ESP_OK;
}

/**
 * @brief Main application
 *
 * Here we open a USB CDC device and send some data to it
 */
void app_main(void)
{
    device_disconnected_sem = xSemaphoreCreateBinary();
    assert(device_disconnected_sem);

    // Install USB Host driver. Should only be called once in entire application
    ESP_LOGI(TAG, "Installing USB Host");
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    // Create a task that will handle USB library events
    BaseType_t task_created = xTaskCreate(usb_lib_task, "usb_lib", 4096, xTaskGetCurrentTaskHandle(), EXAMPLE_USB_HOST_PRIORITY, NULL);
    assert(task_created == pdTRUE);

    ESP_LOGI(TAG, "Installing CDC-ACM driver");
    ESP_ERROR_CHECK(cdc_acm_host_install(NULL));

    const cdc_acm_host_device_config_t dev_config = {
        .connection_timeout_ms = 1000,
        .out_buffer_size = 512,
        .in_buffer_size = 512,
        .user_arg = NULL,
        .event_cb = handle_event,
        .data_cb = handle_rx};

    while (true)
    {

        // Open USB device from tusb_serial_device example example. Either single or dual port configuration.
        ESP_LOGI(TAG, "Opening CDC ACM device 0x%04X:0x%04X...", EXAMPLE_USB_DEVICE_VID, EXAMPLE_USB_DEVICE_PID);
        esp_err_t err = cdc_acm_host_open(EXAMPLE_USB_DEVICE_VID, EXAMPLE_USB_DEVICE_PID, 0, &dev_config, &cdc_dev);
        if (ESP_OK != err)
        {
            ESP_LOGI(TAG, "Failed to open device, retrying...");
            continue;
        }

        if (cdc_dev)
        {
            ESP_LOGI(TAG, "Device opened successfully, handle: %p", cdc_dev);
        }

        cdc_acm_host_desc_print(cdc_dev);
        vTaskDelay(pdMS_TO_TICKS(100));

        ESP_LOGI(TAG, "USB device connected, waiting for Ethernet connection");

        usb_ncm_init(cdc_dev);

        esp_netif_t *test_netif = esp_netif_next(NULL);
        while (test_netif)
        {
            ESP_LOGI(TAG, "Interface: %s", esp_netif_get_desc(test_netif));
            test_netif = esp_netif_next(test_netif);
        }

        // Test sending and receiving: responses are handled in handle_rx callback
        ESP_ERROR_CHECK(cdc_acm_host_data_tx_blocking(cdc_dev, (const uint8_t *)EXAMPLE_TX_STRING, strlen(EXAMPLE_TX_STRING), EXAMPLE_TX_TIMEOUT_MS));
        vTaskDelay(pdMS_TO_TICKS(100));

        // We are done. Wait for device disconnection and start over
        ESP_LOGI(TAG, "Setup success! Waiting for device to disconnect.");
        xSemaphoreTake(device_disconnected_sem, portMAX_DELAY);
    }
}
