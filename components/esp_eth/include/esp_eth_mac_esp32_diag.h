// SMS diagnostic extension for the supported ESP-IDF fork.
#pragma once
#include "esp_eth_mac.h"
#include "esp_eth_tx_diag_stats.h"

#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    esp_eth_tx_diag_stats_t tx;
    uint32_t mac_config, mac_debug, dma_status, dma_operation;
    uint32_t clock_control, phy_interface, tx_current_descriptor;
} esp_eth_mac_esp32_diag_t;

// Task context only. Collects status without changing descriptors, registers,
// interrupt state or MDIO/I2C pins. Register values are best-effort snapshots;
// DMA interrupt flags may already have been acknowledged by the normal ISR.
esp_err_t esp_eth_mac_esp32_get_diagnostics(esp_eth_mac_t *mac,
                                         esp_eth_mac_esp32_diag_t *out);
#ifdef __cplusplus
}
#endif
