#pragma once
#include "Arduino.h"
#include "esp_idf_version.h"
typedef enum { WIFI_SECOND_CHAN_NONE } wifi_second_chan_t;
typedef enum { WIFI_IF_STA } wifi_interface_t;
esp_err_t esp_wifi_set_promiscuous(bool); esp_err_t esp_wifi_set_channel(uint8_t, wifi_second_chan_t);
typedef struct { signed rssi:8; } wifi_pkt_rx_ctrl_t;
typedef struct { const uint8_t *des_addr; } wifi_tx_info_t;
