// Copyright 2019 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <string.h>
#include <stdlib.h>
#include <sys/cdefs.h>
#include "driver/periph_ctrl.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_eth.h"
#include "esp_eth_mac_esp32_diag.h"
#include "esp_pm.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "hal/cpu_hal.h"
#include "hal/emac.h"
#include "soc/soc.h"
#include "sdkconfig.h"
#include "sms_ksz_smi.h"

static const char *TAG = "emac_esp32";
#define MAC_CHECK(a, str, goto_tag, ret_value, ...)                               \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            ret = ret_value;                                                      \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)

#define PHY_OPERATION_TIMEOUT_US (1000)

typedef struct {
    esp_eth_mac_t parent;
    esp_eth_mediator_t *eth;
    emac_hal_context_t hal;
    intr_handle_t intr_hdl;
    SemaphoreHandle_t smi_lock;
    TaskHandle_t rx_task_hdl;
    uint32_t sw_reset_timeout_ms;
    uint32_t frames_remain;
    int smi_mdc_gpio_num;
    int smi_mdio_gpio_num;
    uint8_t addr[6];
    uint8_t *rx_buf[CONFIG_ETH_DMA_RX_BUFFER_NUM];
    uint8_t *tx_buf[CONFIG_ETH_DMA_TX_BUFFER_NUM];
    bool isr_need_yield;
    portMUX_TYPE tx_diag_lock;
    esp_eth_tx_diag_stats_t tx_diag;
    bool tx_diag_pending[CONFIG_ETH_DMA_TX_BUFFER_NUM];
    bool tx_diag_ready;
#ifdef CONFIG_PM_ENABLE
    esp_pm_lock_handle_t pm_lock;
#endif
} emac_esp32_t;

// Descriptor memory is owned by this MAC; no hardware register pointer is
// dereferenced as a software address. The same lock protects reuse and snapshots.
static void emac_esp32_collect_tx(emac_esp32_t *emac)
{
    eth_dma_tx_descriptor_t *base = (eth_dma_tx_descriptor_t *)
        ((uint8_t *)emac->hal.descriptors +
         sizeof(eth_dma_rx_descriptor_t) * CONFIG_ETH_DMA_RX_BUFFER_NUM);
    uint32_t pending = 0;
    for (unsigned i = 0; i < CONFIG_ETH_DMA_TX_BUFFER_NUM; ++i) {
        esp_eth_tx_diag_reap(&emac->tx_diag, &emac->tx_diag_pending[i],
                            base[i].TDES0.Value);
        pending += emac->tx_diag_pending[i];
    }
    emac->tx_diag.pending_desc = pending;
}

static esp_err_t emac_esp32_set_mediator(esp_eth_mac_t *mac, esp_eth_mediator_t *eth)
{
    esp_err_t ret = ESP_OK;
    MAC_CHECK(eth, "can't set mac's mediator to null", err, ESP_ERR_INVALID_ARG);
    emac_esp32_t *emac = __containerof(mac, emac_esp32_t, parent);
    emac->eth = eth;
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_esp32_write_phy_reg_hw(esp_eth_mac_t *mac, uint32_t phy_addr, uint32_t phy_reg, uint32_t reg_value)
{
    esp_err_t ret = ESP_OK;
    emac_esp32_t *emac = __containerof(mac, emac_esp32_t, parent);
    MAC_CHECK(!emac_hal_is_mii_busy(&emac->hal), "phy is busy", err, ESP_ERR_INVALID_STATE);
    emac_hal_set_phy_data(&emac->hal, reg_value);
    emac_hal_set_phy_cmd(&emac->hal, phy_addr, phy_reg, true);
    /* polling the busy flag */
    uint32_t to = 0;
    bool busy = true;
    do {
        ets_delay_us(100);
        busy = emac_hal_is_mii_busy(&emac->hal);
        to += 100;
    } while (busy && to < PHY_OPERATION_TIMEOUT_US);
    MAC_CHECK(!busy, "phy is busy", err, ESP_ERR_TIMEOUT);
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_esp32_read_phy_reg_hw(esp_eth_mac_t *mac, uint32_t phy_addr, uint32_t phy_reg, uint32_t *reg_value)
{
    esp_err_t ret = ESP_OK;
    MAC_CHECK(reg_value, "can't set reg_value to null", err, ESP_ERR_INVALID_ARG);
    emac_esp32_t *emac = __containerof(mac, emac_esp32_t, parent);
    MAC_CHECK(!emac_hal_is_mii_busy(&emac->hal), "phy is busy", err, ESP_ERR_INVALID_STATE);
    emac_hal_set_phy_cmd(&emac->hal, phy_addr, phy_reg, false);
    /* polling the busy flag */
    uint32_t to = 0;
    bool busy = true;
    do {
        ets_delay_us(100);
        busy = emac_hal_is_mii_busy(&emac->hal);
        to += 100;
    } while (busy && to < PHY_OPERATION_TIMEOUT_US);
    MAC_CHECK(!busy, "phy is busy", err, ESP_ERR_TIMEOUT);
    /* Store value */
    *reg_value = emac_hal_get_phy_data(&emac->hal);
    return ESP_OK;
err:
    return ret;
}

static void emac_esp32_init_smi_gpio(emac_esp32_t *emac);

typedef struct { emac_esp32_t *emac; unsigned half_us; } sms_smi_context_t;
static void sms_smi_clock(void *ctx, int level)
{
    sms_smi_context_t *s = ctx;
    gpio_set_level(s->emac->smi_mdc_gpio_num, level);
}
static void sms_smi_data(void *ctx, int level)
{
    sms_smi_context_t *s = ctx;
    gpio_set_level(s->emac->smi_mdio_gpio_num, level);
}
static int sms_smi_sample(void *ctx)
{
    sms_smi_context_t *s = ctx;
    return gpio_get_level(s->emac->smi_mdio_gpio_num);
}
static void sms_smi_delay(void *ctx)
{
    sms_smi_context_t *s = ctx;
    ets_delay_us(s->half_us);
}

/* Same mutex as hardware MIIM; all exit paths restore management pin routing.
   No RMII pin, switch forwarding, speed, duplex, or clock configuration changes. */
static esp_err_t sms_smi_access_profile(emac_esp32_t *emac, uint8_t reg,
    uint32_t *value, bool write, unsigned profile, bool miim, uint8_t phy, bool raw)
{
    if (profile >= SMS_KSZ_V3_PROFILES || !value) return ESP_ERR_INVALID_ARG;
    if (emac_hal_is_mii_busy(&emac->hal)) return ESP_ERR_INVALID_STATE;
    uint32_t mdc_mux = REG_READ(GPIO_PIN_MUX_REG[emac->smi_mdc_gpio_num]);
    uint32_t mdio_mux = REG_READ(GPIO_PIN_MUX_REG[emac->smi_mdio_gpio_num]);
    gpio_set_level(emac->smi_mdc_gpio_num, 0);
    gpio_set_level(emac->smi_mdio_gpio_num, 1);
    gpio_set_direction(emac->smi_mdc_gpio_num, GPIO_MODE_OUTPUT);
    gpio_set_direction(emac->smi_mdio_gpio_num, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_matrix_out(emac->smi_mdc_gpio_num, SIG_GPIO_OUT_IDX, false, false);
    gpio_matrix_out(emac->smi_mdio_gpio_num, SIG_GPIO_OUT_IDX, false, false);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[emac->smi_mdc_gpio_num], PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[emac->smi_mdio_gpio_num], PIN_FUNC_GPIO);
    gpio_set_pull_mode(emac->smi_mdio_gpio_num, GPIO_PULLUP_ONLY);
    sms_smi_context_t context = {emac, sms_ksz_v3_half_us(profile)};
    sms_ksz_smi_bus_t bus = {&context, sms_smi_clock, sms_smi_data, sms_smi_sample, sms_smi_delay};
    uint32_t result = 0;
    bool valid = true;
    if (write) sms_ksz_smi_write(&bus, reg, (uint8_t)*value);
    else {
        result = sms_ksz_v3_capture(&bus, !miim, phy, reg, sms_ksz_v3_phase(profile));
        if (!raw) valid = sms_ksz_v3_decode(result, profile, !miim, &result);
    }
    emac_esp32_init_smi_gpio(emac);
    REG_WRITE(GPIO_PIN_MUX_REG[emac->smi_mdc_gpio_num], mdc_mux);
    REG_WRITE(GPIO_PIN_MUX_REG[emac->smi_mdio_gpio_num], mdio_mux);
    if (!valid) return ESP_ERR_INVALID_RESPONSE;
    if (!write) *value = result;
    return ESP_OK;
}

static esp_err_t emac_esp32_read_phy_reg(esp_eth_mac_t *mac, uint32_t addr,
                                       uint32_t reg, uint32_t *value)
{
    if (!value) return ESP_ERR_INVALID_ARG;
    emac_esp32_t *emac = __containerof(mac, emac_esp32_t, parent);
    if (xSemaphoreTake(emac->smi_lock, pdMS_TO_TICKS(100)) != pdTRUE) return ESP_ERR_TIMEOUT;
    esp_err_t ret;
    if (addr >= SMS_KSZ_V3_SMI_BASE && addr < SMS_KSZ_V3_SMI_BASE + SMS_KSZ_V3_PROFILES && reg <= 0xc6)
        ret = sms_smi_access_profile(emac, (uint8_t)reg, value, false, addr - SMS_KSZ_V3_SMI_BASE, false, 0, false);
    else if (addr >= SMS_KSZ_V3_MIIM_BASE && addr < SMS_KSZ_V3_MIIM_BASE + SMS_KSZ_V3_PROFILES * 2 && reg <= 31)
        ret = sms_smi_access_profile(emac, (uint8_t)reg, value, false, (addr - SMS_KSZ_V3_MIIM_BASE) / 2, true,
                                     (uint8_t)(1 + (addr - SMS_KSZ_V3_MIIM_BASE) % 2), false);
    else if (addr >= SMS_KSZ_V3_RAW_BASE && addr < SMS_KSZ_V3_RAW_BASE + SMS_KSZ_V3_PROFILES && reg <= 0xc6)
        ret = sms_smi_access_profile(emac, (uint8_t)reg, value, false, addr - SMS_KSZ_V3_RAW_BASE, false, 0, true);
    else if (addr == SMS_KSZ_SMI_ADDRESS && reg <= 0xc6)
        ret = sms_smi_access_profile(emac, (uint8_t)reg, value, false, 0, false, 0, false);
    else if (addr <= 31 && reg <= 31)
        ret = emac_esp32_read_phy_reg_hw(mac, addr, reg, value);
    else ret = ESP_ERR_INVALID_ARG;
    xSemaphoreGive(emac->smi_lock);
    return ret;
}

static esp_err_t emac_esp32_write_phy_reg(esp_eth_mac_t *mac, uint32_t addr,
                                        uint32_t reg, uint32_t value)
{
    emac_esp32_t *emac = __containerof(mac, emac_esp32_t, parent);
    if (xSemaphoreTake(emac->smi_lock, pdMS_TO_TICKS(100)) != pdTRUE) return ESP_ERR_TIMEOUT;
    esp_err_t ret;
    if (addr == SMS_KSZ_SMI_ADDRESS || (addr >= SMS_KSZ_V3_SMI_BASE && addr < SMS_KSZ_V3_SMI_BASE + SMS_KSZ_V3_PROFILES)) {
        unsigned profile = addr == SMS_KSZ_SMI_ADDRESS ? 0 : addr - SMS_KSZ_V3_SMI_BASE;
        /* Strict whitelist: only indirect MIB read selection, never forwarding. */
        if ((reg == 0x79 && value == 0x1c) || (reg == 0x7a && value <= 0x5f))
            ret = sms_smi_access_profile(emac, (uint8_t)reg, &value, true, profile, false, 0, false);
        else ret = ESP_ERR_NOT_SUPPORTED;
    } else if (addr <= 31 && reg <= 31)
        ret = emac_esp32_write_phy_reg_hw(mac, addr, reg, value);
    else ret = ESP_ERR_INVALID_ARG;
    xSemaphoreGive(emac->smi_lock);
    return ret;
}

static esp_err_t emac_esp32_set_addr(esp_eth_mac_t *mac, uint8_t *addr)
{
    esp_err_t ret = ESP_OK;
    MAC_CHECK(addr, "can't set mac addr to null", err, ESP_ERR_INVALID_ARG);
    emac_esp32_t *emac = __containerof(mac, emac_esp32_t, parent);
    memcpy(emac->addr, addr, 6);
    emac_hal_set_address(&emac->hal, emac->addr);
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_esp32_get_addr(esp_eth_mac_t *mac, uint8_t *addr)
{
    esp_err_t ret = ESP_OK;
    MAC_CHECK(addr, "can't set mac addr to null", err, ESP_ERR_INVALID_ARG);
    emac_esp32_t *emac = __containerof(mac, emac_esp32_t, parent);
    memcpy(addr, emac->addr, 6);
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_esp32_set_link(esp_eth_mac_t *mac, eth_link_t link)
{
    esp_err_t ret = ESP_OK;
    emac_esp32_t *emac = __containerof(mac, emac_esp32_t, parent);
    switch (link) {
    case ETH_LINK_UP:
        MAC_CHECK(esp_intr_enable(emac->intr_hdl) == ESP_OK, "enable interrupt failed", err, ESP_FAIL);
        emac_hal_start(&emac->hal);
        break;
    case ETH_LINK_DOWN:
        MAC_CHECK(esp_intr_disable(emac->intr_hdl) == ESP_OK, "disable interrupt failed", err, ESP_FAIL);
        emac_hal_stop(&emac->hal);
        break;
    default:
        MAC_CHECK(false, "unknown link status", err, ESP_ERR_INVALID_ARG);
        break;
    }
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_esp32_set_speed(esp_eth_mac_t *mac, eth_speed_t speed)
{
    esp_err_t ret = ESP_OK;
    emac_esp32_t *emac = __containerof(mac, emac_esp32_t, parent);
    switch (speed) {
    case ETH_SPEED_10M:
        emac_hal_set_speed(&emac->hal, EMAC_SPEED_10M);
        ESP_LOGD(TAG, "working in 10Mbps");
        break;
    case ETH_SPEED_100M:
        emac_hal_set_speed(&emac->hal, EMAC_SPEED_100M);
        ESP_LOGD(TAG, "working in 100Mbps");
        break;
    default:
        MAC_CHECK(false, "unknown speed", err, ESP_ERR_INVALID_ARG);
        break;
    }
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_esp32_set_duplex(esp_eth_mac_t *mac, eth_duplex_t duplex)
{
    esp_err_t ret = ESP_OK;
    emac_esp32_t *emac = __containerof(mac, emac_esp32_t, parent);
    switch (duplex) {
    case ETH_DUPLEX_HALF:
        emac_hal_set_duplex(&emac->hal, EMAC_DUPLEX_HALF);
        ESP_LOGD(TAG, "working in half duplex");
        break;
    case ETH_DUPLEX_FULL:
        emac_hal_set_duplex(&emac->hal, EMAC_DUPLEX_FULL);
        ESP_LOGD(TAG, "working in full duplex");
        break;
    default:
        MAC_CHECK(false, "unknown duplex", err, ESP_ERR_INVALID_ARG);
        break;
    }
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_esp32_set_promiscuous(esp_eth_mac_t *mac, bool enable)
{
    emac_esp32_t *emac = __containerof(mac, emac_esp32_t, parent);
    emac_hal_set_promiscuous(&emac->hal, enable);
    return ESP_OK;
}

static esp_err_t emac_esp32_transmit(esp_eth_mac_t *mac, uint8_t *buf, uint32_t length)
{
    esp_err_t ret = ESP_OK;
    emac_esp32_t *emac = __containerof(mac, emac_esp32_t, parent);
    MAC_CHECK(buf && length, "empty TX frame", err, ESP_ERR_INVALID_ARG);
    portENTER_CRITICAL(&emac->tx_diag_lock);
    if (!emac->tx_diag_ready) {
        portEXIT_CRITICAL(&emac->tx_diag_lock);
        return ESP_ERR_INVALID_STATE;
    }
    emac_esp32_collect_tx(emac);
    eth_dma_tx_descriptor_t *first = emac->hal.tx_desc;
    eth_dma_tx_descriptor_t *base = (eth_dma_tx_descriptor_t *)
        ((uint8_t *)emac->hal.descriptors +
         sizeof(eth_dma_rx_descriptor_t) * CONFIG_ETH_DMA_RX_BUFFER_NUM);
    unsigned start = (unsigned)(first - base);
    unsigned used = length / CONFIG_ETH_DMA_BUFFER_SIZE +
                    (length % CONFIG_ETH_DMA_BUFFER_SIZE != 0);
    bool available = used <= CONFIG_ETH_DMA_TX_BUFFER_NUM;
    // DMA can complete AFTER the general sweep. Recheck every candidate here
    // and preserve its final status before allowing HAL to overwrite it.
    // Once CPU-owned, it stays stable until this locked submission hands it
    // back to DMA. If any candidate is still DMA-owned, do not invoke HAL.
    for (unsigned i = 0; available && i < used; ++i) {
        unsigned slot = (start + i) % CONFIG_ETH_DMA_TX_BUFFER_NUM;
        uint32_t status = base[slot].TDES0.Value;
        if (status & (UINT32_C(1) << 31)) {
            available = false;
        } else {
            bool was_pending = emac->tx_diag_pending[slot];
            esp_eth_tx_diag_reap(&emac->tx_diag, &emac->tx_diag_pending[slot], status);
            emac->tx_diag.pending_desc -= was_pending;
        }
    }
    uint32_t sent_len = available ? emac_hal_transmit_frame(&emac->hal, buf, length) : 0;
    if (sent_len == length) {
        for (unsigned i = 0; i < used; ++i)
            emac->tx_diag_pending[(start + i) % CONFIG_ETH_DMA_TX_BUFFER_NUM] = true;
        ++emac->tx_diag.submitted_frames;
        emac->tx_diag.pending_desc += used;
    }
    portEXIT_CRITICAL(&emac->tx_diag_lock);
    MAC_CHECK(sent_len == length, "insufficient TX buffer size", err, ESP_ERR_INVALID_SIZE);
    return ESP_OK;
err:
    return ret;
}

esp_err_t esp_eth_mac_esp32_get_diagnostics(esp_eth_mac_t *mac,
                                         esp_eth_mac_esp32_diag_t *out)
{
    if (!mac || !out || mac->transmit != emac_esp32_transmit)
        return ESP_ERR_INVALID_ARG;
    emac_esp32_t *emac = __containerof(mac, emac_esp32_t, parent);
    portENTER_CRITICAL(&emac->tx_diag_lock);
    if (!emac->tx_diag_ready) {
        portEXIT_CRITICAL(&emac->tx_diag_lock);
        return ESP_ERR_INVALID_STATE;
    }
    emac_esp32_collect_tx(emac);
    out->tx = emac->tx_diag;
    out->mac_config = emac->hal.mac_regs->gmacconfig.val;
    out->mac_debug = emac->hal.mac_regs->emacdebug.val;
    out->dma_status = emac->hal.dma_regs->dmastatus.val;
    out->dma_operation = emac->hal.dma_regs->dmaoperation_mode.val;
    out->clock_control = emac->hal.ext_regs->ex_clk_ctrl.val;
    out->phy_interface = emac->hal.ext_regs->ex_phyinf_conf.val;
    out->tx_current_descriptor = emac->hal.dma_regs->dmatxcurrdesc;
    portEXIT_CRITICAL(&emac->tx_diag_lock);
    return ESP_OK;
}

static esp_err_t emac_esp32_receive(esp_eth_mac_t *mac, uint8_t *buf, uint32_t *length)
{
    esp_err_t ret = ESP_OK;
    uint32_t expected_len = *length;
    emac_esp32_t *emac = __containerof(mac, emac_esp32_t, parent);
    MAC_CHECK(buf && length, "can't set buf and length to null", err, ESP_ERR_INVALID_ARG);
    uint32_t receive_len = emac_hal_receive_frame(&emac->hal, buf, expected_len, &emac->frames_remain);
    /* we need to check the return value in case the buffer size is not enough */
    ESP_LOGD(TAG, "receive len= %d", receive_len);
    MAC_CHECK(expected_len >= receive_len, "received buffer longer than expected", err, ESP_ERR_INVALID_SIZE);
    *length = receive_len;
    return ESP_OK;
err:
    *length = expected_len;
    return ret;
}

static void emac_esp32_rx_task(void *arg)
{
    emac_esp32_t *emac = (emac_esp32_t *)arg;
    uint8_t *buffer = NULL;
    uint32_t length = 0;
    while (1) {
        // block indefinitely until got notification from underlay event
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        do {
            length = ETH_MAX_PACKET_SIZE;
            buffer = malloc(length);
            if (!buffer) {
                ESP_LOGE(TAG, "no mem for receive buffer");
            } else if (emac_esp32_receive(&emac->parent, buffer, &length) == ESP_OK) {
                /* pass the buffer to stack (e.g. TCP/IP layer) */
                if (length) {
                    emac->eth->stack_input(emac->eth, buffer, length);
                } else {
                    free(buffer);
                }
            } else {
                free(buffer);
            }
        } while (emac->frames_remain);
    }
    vTaskDelete(NULL);
}

static void emac_esp32_init_smi_gpio(emac_esp32_t *emac)
{
    /* Setup SMI MDC GPIO */
    gpio_set_direction(emac->smi_mdc_gpio_num, GPIO_MODE_OUTPUT);
    gpio_matrix_out(emac->smi_mdc_gpio_num, EMAC_MDC_O_IDX, false, false);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[emac->smi_mdc_gpio_num], PIN_FUNC_GPIO);
    /* Setup SMI MDIO GPIO */
    gpio_set_direction(emac->smi_mdio_gpio_num, GPIO_MODE_INPUT_OUTPUT);
    gpio_matrix_out(emac->smi_mdio_gpio_num, EMAC_MDO_O_IDX, false, false);
    gpio_matrix_in(emac->smi_mdio_gpio_num, EMAC_MDI_I_IDX, false);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[emac->smi_mdio_gpio_num], PIN_FUNC_GPIO);
}

static esp_err_t emac_esp32_init(esp_eth_mac_t *mac)
{
    esp_err_t ret = ESP_OK;
    emac_esp32_t *emac = __containerof(mac, emac_esp32_t, parent);
    esp_eth_mediator_t *eth = emac->eth;
    /* enable peripheral clock */
    periph_module_enable(PERIPH_EMAC_MODULE);
    /* init clock, config gpio, etc */
    emac_hal_lowlevel_init(&emac->hal);
    /* init gpio used by smi interface */
    emac_esp32_init_smi_gpio(emac);
    MAC_CHECK(eth->on_state_changed(eth, ETH_STATE_LLINIT, NULL) == ESP_OK, "lowlevel init failed", err, ESP_FAIL);
    /* software reset */
    emac_hal_reset(&emac->hal);
    uint32_t to = 0;
    for (to = 0; to < emac->sw_reset_timeout_ms / 10; to++) {
        if (emac_hal_is_reset_done(&emac->hal)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    MAC_CHECK(to < emac->sw_reset_timeout_ms / 10, "reset timeout", err, ESP_ERR_TIMEOUT);
    /* set smi clock */
    emac_hal_set_csr_clock_range(&emac->hal);
    /* reset descriptor chain */
    emac_hal_reset_desc_chain(&emac->hal);
    /* init mac registers by default */
    emac_hal_init_mac_default(&emac->hal);
    /* init dma registers by default */
    emac_hal_init_dma_default(&emac->hal);
    /* Diagnostic bookkeeping only; descriptor contents remain HAL-owned. */
    portENTER_CRITICAL(&emac->tx_diag_lock);
    memset(emac->tx_diag_pending, 0, sizeof(emac->tx_diag_pending));
    emac->tx_diag.pending_desc = 0;
    emac->tx_diag_ready = true;
    portEXIT_CRITICAL(&emac->tx_diag_lock);
    /* get emac address from efuse */
    MAC_CHECK(esp_read_mac(emac->addr, ESP_MAC_ETH) == ESP_OK, "fetch ethernet mac address failed", err, ESP_FAIL);
    /* set MAC address to emac register */
    emac_hal_set_address(&emac->hal, emac->addr);
#ifdef CONFIG_PM_ENABLE
    esp_pm_lock_acquire(emac->pm_lock);
#endif
    return ESP_OK;
err:
    emac->tx_diag_ready = false;
    eth->on_state_changed(eth, ETH_STATE_DEINIT, NULL);
    periph_module_disable(PERIPH_EMAC_MODULE);
    return ret;
}

static esp_err_t emac_esp32_deinit(esp_eth_mac_t *mac)
{
    emac_esp32_t *emac = __containerof(mac, emac_esp32_t, parent);
    esp_eth_mediator_t *eth = emac->eth;
    portENTER_CRITICAL(&emac->tx_diag_lock);
    if (emac->tx_diag_ready) emac_esp32_collect_tx(emac);
    for (unsigned i = 0; i < CONFIG_ETH_DMA_TX_BUFFER_NUM; ++i) {
        emac->tx_diag.abandoned_desc += emac->tx_diag_pending[i];
        emac->tx_diag_pending[i] = false;
    }
    emac->tx_diag.pending_desc = 0;
    emac->tx_diag_ready = false;
    portEXIT_CRITICAL(&emac->tx_diag_lock);
#ifdef CONFIG_PM_ENABLE
    esp_pm_lock_release(emac->pm_lock);
#endif
    emac_hal_stop(&emac->hal);
    eth->on_state_changed(eth, ETH_STATE_DEINIT, NULL);
    periph_module_disable(PERIPH_EMAC_MODULE);
    return ESP_OK;
}

static esp_err_t emac_esp32_start(esp_eth_mac_t *mac)
{
    emac_esp32_t *emac = __containerof(mac, emac_esp32_t, parent);
    emac_hal_start(&emac->hal);
    return ESP_OK;
}

static esp_err_t emac_esp32_stop(esp_eth_mac_t *mac)
{
    emac_esp32_t *emac = __containerof(mac, emac_esp32_t, parent);
    emac_hal_stop(&emac->hal);
    return ESP_OK;
}

static esp_err_t emac_esp32_del(esp_eth_mac_t *mac)
{
    emac_esp32_t *emac = __containerof(mac, emac_esp32_t, parent);
    esp_intr_free(emac->intr_hdl);
    vSemaphoreDelete(emac->smi_lock);
#ifdef CONFIG_PM_ENABLE
    if (emac->pm_lock) {
        esp_pm_lock_delete(emac->pm_lock);
    }
#endif
    vTaskDelete(emac->rx_task_hdl);
    int i = 0;
    for (i = 0; i < CONFIG_ETH_DMA_RX_BUFFER_NUM; i++) {
        free(emac->hal.rx_buf[i]);
    }
    for (i = 0; i < CONFIG_ETH_DMA_TX_BUFFER_NUM; i++) {
        free(emac->hal.tx_buf[i]);
    }
    free(emac->hal.descriptors);
    free(emac);
    return ESP_OK;
}

// To achieve a better performance, we put the ISR always in IRAM
IRAM_ATTR void emac_esp32_isr_handler(void *args)
{
    emac_hal_context_t *hal = (emac_hal_context_t *)args;
    emac_esp32_t *emac = __containerof(hal, emac_esp32_t, hal);
    emac_hal_isr(args);
    if (emac->isr_need_yield) {
        emac->isr_need_yield = false;
        portYIELD_FROM_ISR();
    }
}

esp_eth_mac_t *esp_eth_mac_new_esp32(const eth_mac_config_t *config)
{
    esp_err_t ret_code = ESP_OK;
    esp_eth_mac_t *ret = NULL;
    void *descriptors = NULL;
    emac_esp32_t *emac = NULL;
    MAC_CHECK(config, "can't set mac config to null", err, NULL);
    if (config->flags & ETH_MAC_FLAG_WORK_WITH_CACHE_DISABLE) {
        emac = heap_caps_calloc(1, sizeof(emac_esp32_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    } else {
        emac = calloc(1, sizeof(emac_esp32_t));
    }
    MAC_CHECK(emac, "calloc emac failed", err, NULL);
    emac->tx_diag_lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
    /* alloc memory for ethernet dma descriptor */
    uint32_t desc_size = CONFIG_ETH_DMA_RX_BUFFER_NUM * sizeof(eth_dma_rx_descriptor_t) +
                         CONFIG_ETH_DMA_TX_BUFFER_NUM * sizeof(eth_dma_tx_descriptor_t);
    descriptors = heap_caps_calloc(1, desc_size, MALLOC_CAP_DMA);
    MAC_CHECK(descriptors, "calloc descriptors failed", err, NULL);
    int i = 0;
    /* alloc memory for ethernet dma buffer */
    for (i = 0; i < CONFIG_ETH_DMA_RX_BUFFER_NUM; i++) {
        emac->rx_buf[i] = heap_caps_calloc(1, CONFIG_ETH_DMA_BUFFER_SIZE, MALLOC_CAP_DMA);
        if (!(emac->rx_buf[i])) {
            goto err;
        }
    }
    for (i = 0; i < CONFIG_ETH_DMA_TX_BUFFER_NUM; i++) {
        emac->tx_buf[i] = heap_caps_calloc(1, CONFIG_ETH_DMA_BUFFER_SIZE, MALLOC_CAP_DMA);
        if (!(emac->tx_buf[i])) {
            goto err;
        }
    }
    /* initialize hal layer driver */
    emac_hal_init(&emac->hal, descriptors, emac->rx_buf, emac->tx_buf);
    emac->sw_reset_timeout_ms = config->sw_reset_timeout_ms;
    emac->smi_lock = xSemaphoreCreateMutex();
    MAC_CHECK(emac->smi_lock, "create management mutex failed", err, NULL);
    emac->smi_mdc_gpio_num = config->smi_mdc_gpio_num;
    emac->smi_mdio_gpio_num = config->smi_mdio_gpio_num;
    emac->parent.set_mediator = emac_esp32_set_mediator;
    emac->parent.init = emac_esp32_init;
    emac->parent.deinit = emac_esp32_deinit;
    emac->parent.start = emac_esp32_start;
    emac->parent.stop = emac_esp32_stop;
    emac->parent.del = emac_esp32_del;
    emac->parent.write_phy_reg = emac_esp32_write_phy_reg;
    emac->parent.read_phy_reg = emac_esp32_read_phy_reg;
    emac->parent.set_addr = emac_esp32_set_addr;
    emac->parent.get_addr = emac_esp32_get_addr;
    emac->parent.set_speed = emac_esp32_set_speed;
    emac->parent.set_duplex = emac_esp32_set_duplex;
    emac->parent.set_link = emac_esp32_set_link;
    emac->parent.set_promiscuous = emac_esp32_set_promiscuous;
    emac->parent.transmit = emac_esp32_transmit;
    emac->parent.receive = emac_esp32_receive;
    /* Interrupt configuration */
    if (config->flags & ETH_MAC_FLAG_WORK_WITH_CACHE_DISABLE) {
        ret_code = esp_intr_alloc(ETS_ETH_MAC_INTR_SOURCE, ESP_INTR_FLAG_IRAM,
                                  emac_esp32_isr_handler, &emac->hal, &(emac->intr_hdl));
    } else {
        ret_code = esp_intr_alloc(ETS_ETH_MAC_INTR_SOURCE, 0,
                                  emac_esp32_isr_handler, &emac->hal, &(emac->intr_hdl));
    }
    MAC_CHECK(ret_code == ESP_OK, "alloc emac interrupt failed", err, NULL);
#ifdef CONFIG_PM_ENABLE
    MAC_CHECK(esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "emac_esp32", &emac->pm_lock) == ESP_OK,
              "create pm lock failed", err, NULL);
#endif
    /* create rx task */
    BaseType_t core_num = tskNO_AFFINITY;
    if (config->flags & ETH_MAC_FLAG_PIN_TO_CORE) {
        core_num = cpu_hal_get_core_id();
    }
    BaseType_t xReturned = xTaskCreatePinnedToCore(emac_esp32_rx_task, "emac_rx", config->rx_task_stack_size, emac,
                           config->rx_task_prio, &emac->rx_task_hdl, core_num);
    MAC_CHECK(xReturned == pdPASS, "create emac_rx task failed", err, NULL);
    return &(emac->parent);

err:
    if (emac) {
        if (emac->rx_task_hdl) {
            vTaskDelete(emac->rx_task_hdl);
        }
        if (emac->smi_lock) vSemaphoreDelete(emac->smi_lock);
        if (emac->intr_hdl) {
            esp_intr_free(emac->intr_hdl);
        }
        for (int i = 0; i < CONFIG_ETH_DMA_TX_BUFFER_NUM; i++) {
            free(emac->tx_buf[i]);
        }
        for (int i = 0; i < CONFIG_ETH_DMA_RX_BUFFER_NUM; i++) {
            free(emac->rx_buf[i]);
        }
#ifdef CONFIG_PM_ENABLE
        if (emac->pm_lock) {
            esp_pm_lock_delete(emac->pm_lock);
        }
#endif
        free(emac);
    }
    if (descriptors) {
        free(descriptors);
    }
    return ret;
}

IRAM_ATTR void emac_hal_rx_complete_cb(void *arg)
{
    emac_hal_context_t *hal = (emac_hal_context_t *)arg;
    emac_esp32_t *emac = __containerof(hal, emac_esp32_t, hal);
    BaseType_t high_task_wakeup;
    /* notify receive task */
    vTaskNotifyGiveFromISR(emac->rx_task_hdl, &high_task_wakeup);
    if (high_task_wakeup == pdTRUE) {
        emac->isr_need_yield = true;
    }
}

IRAM_ATTR void emac_hal_rx_unavail_cb(void *arg)
{
    emac_hal_context_t *hal = (emac_hal_context_t *)arg;
    emac_esp32_t *emac = __containerof(hal, emac_esp32_t, hal);
    BaseType_t high_task_wakeup;
    /* notify receive task */
    vTaskNotifyGiveFromISR(emac->rx_task_hdl, &high_task_wakeup);
    if (high_task_wakeup == pdTRUE) {
        emac->isr_need_yield = true;
    }
}

IRAM_ATTR void emac_hal_rx_early_cb(void *arg)
{
    emac_hal_context_t *hal = (emac_hal_context_t *)arg;
    emac_esp32_t *emac = __containerof(hal, emac_esp32_t, hal);
    BaseType_t high_task_wakeup;
    /* notify receive task */
    vTaskNotifyGiveFromISR(emac->rx_task_hdl, &high_task_wakeup);
    if (high_task_wakeup == pdTRUE) {
        emac->isr_need_yield = true;
    }
}
