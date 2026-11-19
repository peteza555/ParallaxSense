#pragma once
#include "esp_wifi.h"
typedef enum { ESP_NOW_SEND_SUCCESS=0, ESP_NOW_SEND_FAIL } esp_now_send_status_t;
typedef struct { uint8_t peer_addr[6]; uint8_t channel; wifi_interface_t ifidx; bool encrypt; } esp_now_peer_info_t;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
typedef struct { uint8_t *src_addr; uint8_t *des_addr; wifi_pkt_rx_ctrl_t *rx_ctrl; } esp_now_recv_info_t;
typedef void (*esp_now_recv_cb_t)(const esp_now_recv_info_t*, const uint8_t*, int);
#else
typedef void (*esp_now_recv_cb_t)(const uint8_t*, const uint8_t*, int);
#endif
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,5,0)
typedef wifi_tx_info_t esp_now_send_info_t;
typedef void (*esp_now_send_cb_t)(const esp_now_send_info_t*, esp_now_send_status_t);
#else
typedef void (*esp_now_send_cb_t)(const uint8_t*, esp_now_send_status_t);
#endif
esp_err_t esp_now_init(); esp_err_t esp_now_register_send_cb(esp_now_send_cb_t); esp_err_t esp_now_register_recv_cb(esp_now_recv_cb_t);
esp_err_t esp_now_add_peer(const esp_now_peer_info_t*); esp_err_t esp_now_send(const uint8_t*, const uint8_t*, size_t);
