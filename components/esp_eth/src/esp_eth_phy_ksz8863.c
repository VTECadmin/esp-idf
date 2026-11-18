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
#include "esp_log.h"
#include "esp_eth.h"
#include "eth_phy_regs_struct.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sms_ksz_smi.h"
#include "sms_ksz_host_mode.h"
static void ksz8863_snapshot_globals(esp_eth_mediator_t *eth);

#define UNIT_TEST false

static const char *TAG = "ksz8863";
#define PHY_CHECK(a, str, goto_tag, ...)                                          \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            ESP_LOGW(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)

/***************Vendor Specific Register***************/

/**
 * @brief LCS(LinkMD Control/Status)
 *
 */
typedef union {
    struct {
		uint32_t vct_fault_count : 9; 	/* Distance to fault */
        uint32_t reserved1 : 3;        	/* Reserved */
		uint32_t vct_10m_short : 1; 	/* Less than 10 meter short */
        uint32_t vct_result : 2;        /* VCT result */
		uint32_t vct_enable : 1; 		/* Enable calbe diagnostic */
    };
    uint32_t val;
} lcs_reg_t;
#define ETH_PHY_LCS_REG_ADDR (0x1D)

/**
 * @brief PSCSR(PHY Special Control Status Register)
 *
 */
typedef union {
    struct {
        uint32_t reserved1 : 1;        	/* Reserved */
        uint32_t remote_loopback : 1; 	/* Remote loopback */
        uint32_t pwrsave : 1;        	/* Power save mode */
        uint32_t force_lnk : 1;   		/* Force link pass */
        uint32_t mdix_status : 1;    	/* Mdi-x */
		uint32_t polrvs	: 1;			/* Polarity reversed */
		uint32_t reserved2 : 10;		/* Reserved */
    };
    uint32_t val;
} pscsr_reg_t;
#define ETH_PHY_PSCSR_REG_ADDR (0x1F)

typedef struct {
    esp_eth_phy_t parent;
    esp_eth_mediator_t *eth;
    const char *name;
    uint32_t addr;
    uint32_t reset_timeout_ms;
    uint32_t autonego_timeout_ms;
    eth_link_t link_status;
	int reset_gpio_num;
    TickType_t last_diagnostic_tick;
} phy_ksz8863_t;

static esp_err_t ksz8863_set_mediator(esp_eth_phy_t *phy, esp_eth_mediator_t *eth)
{
    PHY_CHECK(eth, "can't set mediator for ksz8863 to null", err);
    phy_ksz8863_t *ksz8863 = __containerof(phy, phy_ksz8863_t, parent);
    ksz8863->eth = eth;
    return ESP_OK;
err:
    return ESP_ERR_INVALID_ARG;
}

static void ksz8863_log_status(uint32_t address, bmsr_reg_t status)
{
    if (status.val == 0 || status.val == 0xffff) {
        ESP_LOGW(TAG, "NET PHY%u unusable BMSR=%04x", address, status.val);
    } else {
        ESP_LOGI(TAG, "NET PHY%u BMSR=%04x link_sample=%u autoneg_sample=%u",
                 address, status.val, status.link_status, status.auto_nego_complete);
    }
}

static esp_err_t ksz8863_get_link(esp_eth_phy_t *phy)
{
    phy_ksz8863_t *ksz8863 = __containerof(phy, phy_ksz8863_t, parent);
    esp_eth_mediator_t *eth = ksz8863->eth;
    ksz8863_snapshot_globals(eth);
    bmsr_reg_t bmsr;
    #if UNIT_TEST==false
    bmsr_reg_t bmsr1;
    #endif
    PHY_CHECK(eth->phy_reg_read(eth, ksz8863->addr, ETH_PHY_BMSR_REG_ADDR, &(bmsr.val)) == ESP_OK, "read BMSR failed", err);
    #if UNIT_TEST==false
    PHY_CHECK(eth->phy_reg_read(eth, (ksz8863->addr + 1), ETH_PHY_BMSR_REG_ADDR, &(bmsr1.val)) == ESP_OK, "read BMSR failed", err);
    #endif
    eth_link_t link = bmsr.link_status ? ETH_LINK_UP : ETH_LINK_DOWN;
    #if UNIT_TEST==false
    eth_link_t link1 = bmsr1.link_status ? ETH_LINK_UP : ETH_LINK_DOWN;
    #endif
    eth_link_t masterlink;
    // Reuse samples from the existing polling path. A second MDIO caller would
    // race the MAC's unprotected busy-check/command sequence. BMSR link is
    // latch-low, so the next poll can differ after a transient link loss.
    TickType_t now = xTaskGetTickCount();
    if ((TickType_t)(now - ksz8863->last_diagnostic_tick) >= pdMS_TO_TICKS(30000)) {
        ksz8863->last_diagnostic_tick = now;
        ksz8863_log_status(ksz8863->addr, bmsr);
        #if UNIT_TEST==false
        ksz8863_log_status(ksz8863->addr + 1, bmsr1);
        #endif
    }
    if(link == ETH_LINK_UP)
	{
		masterlink = ETH_LINK_UP;
	}
    #if UNIT_TEST==false
	else if(link1 == ETH_LINK_UP)
	{
		masterlink = ETH_LINK_UP;
	}
    #endif
	else
	{
        masterlink = ETH_LINK_DOWN;
	}

    // ESP_LOGI(TAG, "bmsr0: %d", bmsr.link_status);
    // ESP_LOGI(TAG, "bmsr1: %d", bmsr1.link_status);

    if (ksz8863->link_status != masterlink) {
        if (link == ETH_LINK_UP) {
            if(ksz8863->addr == 0x1)
            ESP_LOGI(TAG, "Link from 1");
            else if(ksz8863->addr == 0x2)
            ESP_LOGI(TAG, "Link from 2");
            phy->negotiate(phy,0);
        }
        #if UNIT_TEST==false
        else if (link1 == ETH_LINK_UP) {
            ESP_LOGI(TAG, "Link from 2");
            phy->negotiate(phy,1);
        }
        #endif
        else {
            PHY_CHECK(eth->on_state_changed(eth, ETH_STATE_LINK, (void *)link) == ESP_OK, "send link event failed", err);
            ksz8863->link_status = link;
        }
    }
    return ESP_OK;
err:
    return ESP_FAIL;
}

static esp_err_t ksz8863_reset_hw(esp_eth_phy_t *phy)
{
    phy_ksz8863_t *ksz8863 = __containerof(phy, phy_ksz8863_t, parent);
    if (ksz8863->reset_gpio_num >= 0) {
        gpio_pad_select_gpio(ksz8863->reset_gpio_num);
        gpio_set_direction(ksz8863->reset_gpio_num, GPIO_MODE_OUTPUT);
        gpio_set_level(ksz8863->reset_gpio_num, 0);
        gpio_set_level(ksz8863->reset_gpio_num, 1);
    }
    return ESP_OK;
}

static esp_err_t ksz8863_reset(esp_eth_phy_t *phy)
{
    phy_ksz8863_t *ksz8863 = __containerof(phy, phy_ksz8863_t, parent);
    esp_eth_mediator_t *eth = ksz8863->eth;
    bmcr_reg_t bmcr = {.reset = 1};
    PHY_CHECK(eth->phy_reg_write(eth, ksz8863->addr, ETH_PHY_BMCR_REG_ADDR, bmcr.val) == ESP_OK, "write BMCR failed", err);
    /* wait for reset complete */
    uint32_t to = 0;
    for (to = 0; to < ksz8863->reset_timeout_ms / 10; to++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        PHY_CHECK(eth->phy_reg_read(eth, ksz8863->addr, ETH_PHY_BMCR_REG_ADDR, &(bmcr.val)) == ESP_OK, "read BMCR failed", err);
        if (!bmcr.reset) {
            break;
        }
    }
    PHY_CHECK(to < ksz8863->reset_timeout_ms / 10, "PHY reset timeout", err);
    return ESP_OK;
err:
    return ESP_FAIL;
}

static esp_err_t ksz8863_negotiate(esp_eth_phy_t *phy, uint8_t address)
{
    ESP_LOGI(TAG, "Autonegotiate");
	phy_ksz8863_t *ksz8863 = __containerof(phy, phy_ksz8863_t, parent);
    esp_eth_mediator_t *eth = ksz8863->eth;
    /* Start auto negotiation */
    bmcr_reg_t bmcr = {
        .speed_select = 1,     /* 100Mbps */
        .duplex_mode = 1,      /* Full Duplex */
        .en_auto_nego = 1,     /* Auto Negotiation */
        .restart_auto_nego = 1 /* Restart Auto Negotiation */
    };
    PHY_CHECK(eth->phy_reg_write(eth, (ksz8863->addr + address), ETH_PHY_BMCR_REG_ADDR, bmcr.val) == ESP_OK, "write BMCR failed", err);
    /* Wait for auto negotiation complete */
    bmsr_reg_t bmsr;
    uint32_t to = 0;
    for (to = 0; to < ksz8863->autonego_timeout_ms / 10; to++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        PHY_CHECK(eth->phy_reg_read(eth, (ksz8863->addr + address), ETH_PHY_BMSR_REG_ADDR, &(bmsr.val)) == ESP_OK, "read BMSR failed", err);
        if (bmsr.auto_nego_complete) {
            break;
        }
    }
    /* Auto negotiation failed, maybe no network cable plugged in, so output a warning */
    if (to >= ksz8863->autonego_timeout_ms / 10) {
        ESP_LOGW(TAG, "Ethernet PHY auto negotiation timeout");
    }
    /* Update information about link, speed, duplex */
    PHY_CHECK(eth->phy_reg_read(eth, (ksz8863->addr + address), ETH_PHY_BMSR_REG_ADDR, &(bmsr.val)) == ESP_OK, "read BMSR failed", err);
    eth_link_t link = bmsr.link_status ? ETH_LINK_UP : ETH_LINK_DOWN;
    /* Copper PHYs negotiate independently; this event configures the fixed
       ESP32 <-> KSZ port-3 RMII host link, not the copper port's result. */
    eth_speed_t speed = SMS_KSZ_HOST_SPEED;
    eth_duplex_t duplex = SMS_KSZ_HOST_DUPLEX;
    PHY_CHECK(eth->on_state_changed(eth, ETH_STATE_SPEED, (void *)speed) == ESP_OK, "send speed event failed", err);
    PHY_CHECK(eth->on_state_changed(eth, ETH_STATE_DUPLEX, (void *)duplex) == ESP_OK, "send duplex event failed", err);
    ESP_LOGI(TAG, "NET RMII host requested=%uMbps/full; verify G06=%02x and live EMAC speed/duplex bits",
             SMS_KSZ_HOST_SPEED_MBPS, SMS_KSZ_HOST_CONTROL4);
    if (ksz8863->link_status != link) {
        PHY_CHECK(eth->on_state_changed(eth, ETH_STATE_LINK, (void *)link) == ESP_OK, "send link event failed", err);
        ksz8863->link_status = link;
    }
    return ESP_OK;
err:
    return ESP_FAIL;
}

static esp_err_t ksz8863_pwrctl(esp_eth_phy_t *phy, bool enable)
{
    phy_ksz8863_t *ksz8863 = __containerof(phy, phy_ksz8863_t, parent);
    esp_eth_mediator_t *eth = ksz8863->eth;
    bmcr_reg_t bmcr;
    PHY_CHECK(eth->phy_reg_read(eth, ksz8863->addr, ETH_PHY_BMCR_REG_ADDR, &(bmcr.val)) == ESP_OK, "read BMCR failed", err);
    ESP_LOGI(TAG, "BMCR: %d", bmcr.val);
	if (!enable) {
        /* Enable IEEE Power Down Mode */
        bmcr.power_down = 1;
    } else {
        /* Disable IEEE Power Down Mode */
        bmcr.power_down = 0;
    }
    PHY_CHECK(eth->phy_reg_write(eth, ksz8863->addr, ETH_PHY_BMCR_REG_ADDR, bmcr.val) == ESP_OK, "write BMCR failed", err);
    PHY_CHECK(eth->phy_reg_read(eth, ksz8863->addr, ETH_PHY_BMCR_REG_ADDR, &(bmcr.val)) == ESP_OK, "read BMCR failed", err);
    ESP_LOGI(TAG, "BMCR: %d", bmcr.val);
	if (!enable) {
        PHY_CHECK(bmcr.power_down == 1, "power down failed", err);
    } else {
        PHY_CHECK(bmcr.power_down == 0, "power up failed", err);
    }
    return ESP_OK;
err:
    return ESP_FAIL;
}

static esp_err_t ksz8863_set_addr(esp_eth_phy_t *phy, uint32_t addr)
{
    phy_ksz8863_t *ksz8863 = __containerof(phy, phy_ksz8863_t, parent);
    ksz8863->addr = addr;
    return ESP_OK;
}

static esp_err_t ksz8863_get_addr(esp_eth_phy_t *phy, uint32_t *addr)
{
    PHY_CHECK(addr, "get phy address failed", err);
    phy_ksz8863_t *ksz8863 = __containerof(phy, phy_ksz8863_t, parent);
    *addr = ksz8863->addr;
    return ESP_OK;
err:
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t ksz8863_del(esp_eth_phy_t *phy)
{
    phy_ksz8863_t *ksz8863 = __containerof(phy, phy_ksz8863_t, parent);
    free(ksz8863);
    return ESP_OK;
}

/* v3 survey is read-only, including on a failing initialization path. */
static int ksz_v3_profile = -1;
static TickType_t ksz_v3_last_tick;

static esp_err_t ksz8863_v3_read(esp_eth_mediator_t *eth, unsigned profile,
                               unsigned reference, uint32_t *value)
{
    const uint8_t regs[] = {0x00, 0x01, 0x06, 0x35, 2, 3, 2, 3};
    uint32_t address = reference < 4 ? SMS_KSZ_V3_SMI_BASE + profile :
        SMS_KSZ_V3_MIIM_BASE + profile * 2U + (reference >= 6 ? 1U : 0U);
    return eth->phy_reg_read(eth, address, regs[reference], value);
}

static void ksz8863_probe_v3(esp_eth_mediator_t *eth, const char *stage)
{
    const uint32_t expected[] = {0x88, 0x31, SMS_KSZ_HOST_CONTROL4, 0x80, 0x22, 0x1430, 0x22, 0x1430};
    const uint32_t masks[] = {0xff, 0xf1, 0xff, 0xff, 0xffff, 0xfff0, 0xffff, 0xfff0};
    uint32_t passing = 0;
    ksz_v3_profile = -1;
    ESP_LOGI(TAG, "NET KSZ v3 survey stage=%s: read-only; G00/G01/G06/P35 and both PHY IDs, 3 repetitions", stage);
    /* Capture hardware MIIM BEFORE bit-banging/remuxing for comparison. */
    for (unsigned port = 1; port <= 2; ++port) {
        uint32_t id1 = 0, id2 = 0;
        esp_err_t one = eth->phy_reg_read(eth, port, ETH_PHY_IDR1_REG_ADDR, &id1);
        esp_err_t two = eth->phy_reg_read(eth, port, ETH_PHY_IDR2_REG_ADDR, &id2);
        ESP_LOGI(TAG, "NET KSZ v3 hardware-before-remux PHY%u=%04x/%04x result=%s/%s",
                 port, id1, id2, esp_err_to_name(one), esp_err_to_name(two));
    }
    for (unsigned profile = 0; profile < SMS_KSZ_V3_PROFILES; ++profile) {
        unsigned matches = 0, errors = 0;
        bool stable = true;
        uint32_t values[8] = {0}, raw = 0;
        for (unsigned repeat = 0; repeat < 3; ++repeat) {
            for (unsigned ref = 0; ref < 8; ++ref) {
                uint32_t value = 0;
                esp_err_t err = ksz8863_v3_read(eth, profile, ref, &value);
                if (err != ESP_OK) ++errors;
                else if ((value & masks[ref]) == expected[ref]) ++matches;
                if (repeat && value != values[ref]) stable = false;
                values[ref] = value;
            }
        }
        esp_err_t raw_err = eth->phy_reg_read(eth, SMS_KSZ_V3_RAW_BASE + profile, 0, &raw);
        if (matches == 24 && errors == 0 && stable) passing |= 1U << profile;
        ESP_LOGI(TAG, "NET KSZ v3 p=%u %s matches=%u/24 errors=%u stable=%d G00/01/06/P35=%02x/%02x/%02x/%02x PHY1=%04x/%04x PHY2=%04x/%04x raw18_G00=%05x(%s)",
                 profile, sms_ksz_v3_name(profile), matches, errors, stable,
                 values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7],
                 raw, esp_err_to_name(raw_err));
    }
    ksz_v3_profile = sms_ksz_v3_select(passing);
    ksz_v3_last_tick = xTaskGetTickCount();
    if (ksz_v3_profile < 0)
        ESP_LOGW(TAG, "NET KSZ v3 UNTRUSTED: no profile passed all references; no MIB selector writes or counter claims");
    else
        ESP_LOGI(TAG, "NET KSZ v3 validated_mask=%02x selected=%d %s; MIB requires selector readback and reference recheck",
                 passing, ksz_v3_profile, sms_ksz_v3_name((unsigned)ksz_v3_profile));
}

static esp_err_t ksz8863_stable_phy_id(esp_eth_mediator_t *eth, uint32_t addr,
                                     uint32_t *id1, uint32_t *id2)
{
    unsigned stable = 0;
    uint32_t previous = 0;
    for (unsigned attempt = 0; attempt < 8; ++attempt) {
        uint32_t one = 0, two = 0;
        esp_err_t err = eth->phy_reg_read(eth, addr, ETH_PHY_IDR1_REG_ADDR, &one);
        if (err == ESP_OK) err = eth->phy_reg_read(eth, addr, ETH_PHY_IDR2_REG_ADDR, &two);
        bool valid = err == ESP_OK && one == 0x22 && (two & 0xfff0) == 0x1430;
        stable = valid ? (stable && two == previous ? stable + 1 : 1) : 0;
        previous = two;
        if (stable == 3) {
            *id1 = one; *id2 = two;
            ESP_LOGI(TAG, "NET KSZ v3 hardware PHY ID stable=3 phy=%u raw=%04x/%04x attempt=%u", addr, one, two, attempt + 1);
            return ESP_OK;
        }
        if (!valid) ESP_LOGW(TAG, "NET KSZ v3 hardware PHY ID unstable phy=%u attempt=%u raw=%04x/%04x result=%s",
                            addr, attempt + 1, one, two, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGW(TAG, "NET KSZ v3 PHY identification remains unconfirmed; startup validation is advisory");
    return ESP_ERR_INVALID_RESPONSE;
}

static esp_err_t ksz8863_init(esp_eth_phy_t *phy)
{
    if (!phy) return ESP_ERR_INVALID_ARG;
    phy_ksz8863_t *ksz8863 = __containerof(phy, phy_ksz8863_t, parent);
    esp_eth_mediator_t *eth = ksz8863->eth;
    if (!eth) return ESP_ERR_INVALID_STATE;
    ksz8863_probe_v3(eth, "boot-before-PHY-ID");
    // The switch was configured by the board startup code. An unavailable
    // management read does not prove the forwarding/data path is unusable.
    esp_err_t err = ksz8863_pwrctl(phy, true);
    if (err != ESP_OK) ESP_LOGW(TAG, "KSZ PHY power-state request unconfirmed: %s; continuing", esp_err_to_name(err));
    uint32_t id1 = 0, id2 = 0;
    err = ksz8863_stable_phy_id(eth, ksz8863->addr, &id1, &id2);
    if (err != ESP_OK) ESP_LOGW(TAG, "KSZ PHY ID validation warning: %s; continuing driver initialization", esp_err_to_name(err));
    const uint8_t regs[] = {ETH_PHY_BMCR_REG_ADDR, ETH_PHY_BMSR_REG_ADDR,
        ETH_PHY_ANAR_REG_ADDR, ETH_PHY_ANLPAR_REG_ADDR, ETH_PHY_ANER_REG_ADDR,
        ETH_PHY_LCS_REG_ADDR, ETH_PHY_PSCSR_REG_ADDR};
    for (unsigned i = 0; i < sizeof(regs); ++i) {
        uint32_t value = 0;
        err = eth->phy_reg_read(eth, ksz8863->addr, regs[i], &value);
        if (err != ESP_OK) ESP_LOGW(TAG, "KSZ PHY startup read warning: reg=0x%02x read=%s; continuing", regs[i], esp_err_to_name(err));
        else ESP_LOGI(TAG, "KSZ PHY startup: phy=%u reg=0x%02x raw=0x%04x", ksz8863->addr, regs[i], value);
    }
    return ESP_OK;
}

static esp_err_t ksz8863_deinit(esp_eth_phy_t *phy)
{
    /* Power off Ethernet PHY */
    PHY_CHECK(ksz8863_pwrctl(phy, true) == ESP_OK, "power off Ethernet PHY failed", err);
    return ESP_OK;
err:
    return ESP_FAIL;
}

esp_eth_phy_t *esp_eth_phy_new_ksz8863(const eth_phy_config_t *config)
{
    PHY_CHECK(config, "can't set phy config to null", err);
    phy_ksz8863_t *ksz8863 = calloc(1, sizeof(phy_ksz8863_t));
    PHY_CHECK(ksz8863, "calloc ksz8863 object failed", err);
    ksz8863->name = "ksz8863";
    ksz8863->addr = config->phy_addr;
    ksz8863->reset_timeout_ms = config->reset_timeout_ms;
    ksz8863->reset_gpio_num = config->reset_gpio_num;
    ksz8863->link_status = ETH_LINK_DOWN;
    ksz8863->autonego_timeout_ms = config->autonego_timeout_ms;
    ksz8863->parent.reset = ksz8863_reset;
	ksz8863->parent.reset_hw = ksz8863_reset_hw;
    ksz8863->parent.init = ksz8863_init;
    ksz8863->parent.deinit = ksz8863_deinit;
    ksz8863->parent.set_mediator = ksz8863_set_mediator;
    ksz8863->parent.negotiate = ksz8863_negotiate;
    ksz8863->parent.get_link = ksz8863_get_link;
    ksz8863->parent.pwrctl = ksz8863_pwrctl;
    ksz8863->parent.get_addr = ksz8863_get_addr;
    ksz8863->parent.set_addr = ksz8863_set_addr;
    ksz8863->parent.del = ksz8863_del;

    return &(ksz8863->parent);
err:
    return NULL;
}

// Called from the existing PHY poller, never from a competing application task.
// Only MIB indirect-read selector writes are issued; counter reads clear those
// per-port counters, so logged values are intervals, not lifetime totals.
static esp_err_t ksz8863_global_read(esp_eth_mediator_t *eth, uint8_t reg, uint32_t *value)
{
    if (ksz_v3_profile < 0) return ESP_ERR_INVALID_STATE;
    return eth->phy_reg_read(eth, SMS_KSZ_V3_SMI_BASE + (unsigned)ksz_v3_profile, reg, value);
}

static esp_err_t ksz8863_mib_read(esp_eth_mediator_t *eth, uint8_t index, uint32_t *value, bool *overflow)
{
    if (ksz_v3_profile < 0) return ESP_ERR_INVALID_STATE;
    uint32_t access = SMS_KSZ_V3_SMI_BASE + (unsigned)ksz_v3_profile;
    esp_err_t err = eth->phy_reg_write(eth, access, 0x79, 0x1c);
    if (err != ESP_OK) return err;
    uint32_t selector = 0;
    err = ksz8863_global_read(eth, 0x79, &selector);
    if (err != ESP_OK || selector != 0x1c) return ESP_ERR_INVALID_RESPONSE;
    err = eth->phy_reg_write(eth, access, 0x7a, index);
    if (err != ESP_OK) return err;
    err = ksz8863_global_read(eth, 0x7a, &selector);
    if (err != ESP_OK || selector != index) return ESP_ERR_INVALID_RESPONSE;
    uint32_t high = 0;
    for (unsigned attempt = 0; attempt < 3; ++attempt) {
        err = ksz8863_global_read(eth, 0x80, &high);
        if (err != ESP_OK) return err;
        if (high & 0x40) break;
    }
    if (!(high & 0x40)) return ESP_ERR_TIMEOUT;
    *overflow = (high & 0x80) != 0;
    uint32_t result = high & 0x3f;
    for (unsigned reg = 0x81; reg <= 0x83; ++reg) {
        uint32_t byte;
        err = ksz8863_global_read(eth, (uint8_t)reg, &byte);
        if (err != ESP_OK) return err;
        result = (result << 8) | byte;
    }
    *value = result;
    return ESP_OK;
}

static void ksz8863_snapshot_globals(esp_eth_mediator_t *eth)
{
    TickType_t now = xTaskGetTickCount();
    if ((TickType_t)(now - ksz_v3_last_tick) < pdMS_TO_TICKS(10 * 60 * 1000)) return; /* Periodic survey every 10 minutes. */
    ksz8863_probe_v3(eth, "periodic");
    if (ksz_v3_profile < 0) return;
    uint32_t host = 0;
    esp_err_t err = ESP_OK;
    if (ksz8863_global_read(eth, 0xc6, &host) != ESP_OK) return;
    ESP_LOGI(TAG, "NET KSZ v3 C6=%02x host_mode=%u P3_internal_clock=%u", host, host & 3, (host >> 3) & 1);
    const uint8_t regs[] = {0x03,0x05,0x06,0x10,0x11,0x12,0x20,0x21,0x22,0x30,0x31,0x32,0x35,0x3f,0xa6,0xc3};
    for (unsigned i = 0; i < sizeof(regs); ++i) {
        uint32_t byte = 0;
        err = ksz8863_global_read(eth, regs[i], &byte);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "NET KSZ global reg[%02x] failed: %s", regs[i], esp_err_to_name(err));
            return;
        }
        ESP_LOGI(TAG, "NET KSZ global reg[%02x]=%02x", regs[i], byte);
    }
    for (unsigned port = 1; port <= 2; ++port) {
        uint32_t status0 = 0, status1 = 0;
        uint8_t reg = port == 1 ? 0x1e : 0x2e;
        if (ksz8863_global_read(eth, reg, &status0) != ESP_OK ||
            ksz8863_global_read(eth, reg + 1, &status1) != ESP_OK) {
            ESP_LOGW(TAG, "NET copper P%u status read failed; no negotiated-mode claim", port);
            return;
        }
        if (status0 & 0x20)
            ESP_LOGI(TAG, "NET copper P%u link=1 AN_done=%u speed=%uMbps duplex=%s raw=%02x/%02x",
                     port, (status0 >> 6) & 1, status1 & 4 ? 100 : 10,
                     status1 & 2 ? "full" : "half", status0, status1);
        else ESP_LOGI(TAG, "NET copper P%u link=0 speed/duplex=unavailable raw=%02x/%02x", port, status0, status1);
    }
    // Per-port table offsets: good broadcasts, CRC/alignment/symbol errors,
    // fragments, and good transmitted broadcasts. Port bases are 00/20/40.
    const uint8_t offsets[] = {0x0b,0x07,0x08,0x06,0x03,0x18};
    uint32_t samples[3][6] = {{0}}, overflow_masks[3] = {0};
    for (unsigned port = 0; port < 3; ++port) {
        uint32_t *values = samples[port];
        for (unsigned i = 0; i < sizeof(offsets); ++i) {
            bool overflow = false;
            err = ksz8863_mib_read(eth, (uint8_t)(port * 0x20 + offsets[i]), &values[i], &overflow);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "NET KSZ MIB port=%u counter=%02x failed: %s (partial sample)",
                         port + 1, offsets[i], esp_err_to_name(err));
                return;
            }
            if (overflow) overflow_masks[port] |= 1U << i;
        }
    }
    const uint32_t expected[] = {0x88,0x31,SMS_KSZ_HOST_CONTROL4,0x80,0x22,0x1430,0x22,0x1430};
    const uint32_t masks[] = {0xff,0xf1,0xff,0xff,0xffff,0xfff0,0xffff,0xfff0};
    for (unsigned ref = 0; ref < 8; ++ref) {
        uint32_t value = 0;
        esp_err_t check = ksz8863_v3_read(eth, (unsigned)ksz_v3_profile, ref, &value);
        if (check != ESP_OK || (value & masks[ref]) != expected[ref]) {
            ESP_LOGW(TAG, "NET KSZ v3 post-MIB reference failed ref=%u raw=%04x; discarded interval, counters may have cleared", ref, value);
            ksz_v3_profile = -1;
            return;
        }
    }
    for (unsigned port = 0; port < 3; ++port) {
        uint32_t *values = samples[port];
        ESP_LOGI(TAG, "NET KSZ MIB v3 validated interval P%u rx_bcast=%u rx_crc=%u rx_align=%u rx_symbol=%u rx_frag=%u tx_bcast=%u overflow_mask=%02x",
                 port + 1, values[0],values[1],values[2],values[3],values[4],values[5],overflow_masks[port]);
    }
}
