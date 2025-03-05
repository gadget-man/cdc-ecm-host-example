/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"

#include "esp_event.h"
#include "esp_netif.h"
#include "esp_eth.h"

#include "cdc_ecm_host.h"

#define EXAMPLE_USB_DEVICE_VID (0x0BDA)
#define EXAMPLE_USB_DEVICE_PID (0x8152)
#define EXAMPLE_USB_DEVICE_PID_2 (0x8153)

// Define the maximum size for an Ethernet frame (including the FCS)
#define MAX_ETHERNET_FRAME_SIZE 1518

static const char *TAG = "main";

/** Event handler for Ethernet events */
static void netif_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == ETH_EVENT)
    {

        switch (event_id)
        {
        case ETHERNET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Ethernet Connected, waiting for IP ...");
            // esp_netif_t *netif = esp_netif_get_handle_from_ifkey("cdc_ecm_host");
            // const char *hostname;
            // esp_err_t err = esp_netif_get_hostname(netif, &hostname);
            // if (err == ESP_OK)
            // {
            //     ESP_LOGI(TAG, "ESP Hostname: %s", hostname);
            // }
            // else
            // {
            //     ESP_LOGE(TAG, "Failed to get hostname, error: %s", esp_err_to_name(err));
            // }
            break;
        case ETHERNET_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "Ethernet Disconnected");
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
            ESP_LOGI(TAG, "Got IP: " IPSTR ", Subnet Mask: " IPSTR ", Gateway: " IPSTR, IP2STR(&event->ip_info.ip), IP2STR(&event->ip_info.netmask), IP2STR(&event->ip_info.gw));
            break;
        }
        case IP_EVENT_ETH_LOST_IP:
            ESP_LOGI(TAG, "Ethernet lost IP address");
            break;
        default:
            // ESP_LOGW(TAG, "Unhandled IP Event: %ld", event_id);
            break;
        }
    }
}

// Statically allocated parameters for the task.
static cdc_ecm_params_t cdc_ecm_params = {
    .vid = EXAMPLE_USB_DEVICE_VID,
    .pids = {EXAMPLE_USB_DEVICE_PID, EXAMPLE_USB_DEVICE_PID_2},
    .event_cb = netif_event_handler,
    .callback_arg = NULL,
    // .if_key = "cdc_ecm_host",
    // .if_desc = "usb cdc ecm host device",
    // .hostname = "Espressif CDC-ECM Host Device"
};

/**
 * @brief Main application
 *
 */
void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("USB-CDC", ESP_LOG_DEBUG);
    // esp_log_level_set("cdc_ecm", ESP_LOG_DEBUG);

    cdc_ecm_init(&cdc_ecm_params);
}
