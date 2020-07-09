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

static const char *TAG = "ksz8863";
#define PHY_CHECK(a, str, goto_tag, ...)                                          \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
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

static esp_err_t ksz8863_get_link(esp_eth_phy_t *phy)
{
    phy_ksz8863_t *ksz8863 = __containerof(phy, phy_ksz8863_t, parent);
    esp_eth_mediator_t *eth = ksz8863->eth;
    bmsr_reg_t bmsr;

    PHY_CHECK(eth->phy_reg_read(eth, ksz8863->addr, ETH_PHY_BMSR_REG_ADDR, &(bmsr.val)) == ESP_OK, "read BMSR failed", err);
    eth_link_t link = bmsr.link_status ? ETH_LINK_UP : ETH_LINK_DOWN;
    if (ksz8863->link_status != link) {
        if (link == ETH_LINK_UP) {
            phy->negotiate(phy);
        } else {
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

static esp_err_t ksz8863_negotiate(esp_eth_phy_t *phy)
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
    PHY_CHECK(eth->phy_reg_write(eth, ksz8863->addr, ETH_PHY_BMCR_REG_ADDR, bmcr.val) == ESP_OK, "write BMCR failed", err);
    /* Wait for auto negotiation complete */
    bmsr_reg_t bmsr;
    uint32_t to = 0;
    for (to = 0; to < ksz8863->autonego_timeout_ms / 10; to++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        PHY_CHECK(eth->phy_reg_read(eth, ksz8863->addr, ETH_PHY_BMSR_REG_ADDR, &(bmsr.val)) == ESP_OK, "read BMSR failed", err);
        if (bmsr.auto_nego_complete) {
            break;
        }
    }
    /* Auto negotiation failed, maybe no network cable plugged in, so output a warning */
    if (to >= ksz8863->autonego_timeout_ms / 10) {
        ESP_LOGW(TAG, "Ethernet PHY auto negotiation timeout");
    }
    /* Update information about link, speed, duplex */
    PHY_CHECK(eth->phy_reg_read(eth, ksz8863->addr, ETH_PHY_BMSR_REG_ADDR, &(bmsr.val)) == ESP_OK, "read BMSR failed", err);
    eth_link_t link = bmsr.link_status ? ETH_LINK_UP : ETH_LINK_DOWN;
    eth_speed_t speed = ETH_SPEED_10M;
    eth_duplex_t duplex = ETH_DUPLEX_HALF;
    PHY_CHECK(eth->on_state_changed(eth, ETH_STATE_SPEED, (void *)speed) == ESP_OK, "send speed event failed", err);
    PHY_CHECK(eth->on_state_changed(eth, ETH_STATE_DUPLEX, (void *)duplex) == ESP_OK, "send duplex event failed", err);
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

static esp_err_t ksz8863_init(esp_eth_phy_t *phy)
{
    phy_ksz8863_t *ksz8863 = __containerof(phy, phy_ksz8863_t, parent);
    esp_eth_mediator_t *eth = ksz8863->eth;
    /* Power on Ethernet PHY */
    PHY_CHECK(ksz8863_pwrctl(phy, true) == ESP_OK, "power on Ethernet PHY failed", err);
    /* Reset Ethernet PHY */
    PHY_CHECK(ksz8863_reset(phy) == ESP_OK, "reset Ethernet PHY failed", err);
    /* Check PHY ID */
    phyidr1_reg_t id1;
    phyidr2_reg_t id2;
	bmcr_reg_t bmcr;
    PHY_CHECK(eth->phy_reg_read(eth, ksz8863->addr, ETH_PHY_IDR1_REG_ADDR, &(id1.val)) == ESP_OK, "read ID1 failed", err);
    PHY_CHECK(eth->phy_reg_read(eth, ksz8863->addr, ETH_PHY_IDR2_REG_ADDR, &(id2.val)) == ESP_OK, "read ID2 failed", err);
	PHY_CHECK(eth->phy_reg_read(eth, ksz8863->addr, ETH_PHY_BMCR_REG_ADDR, &(bmcr.val)) == ESP_OK, "read BMCR failed", err);
	ESP_LOGI(TAG, "BMCR: %d", bmcr.val);
	bmsr_reg_t bmsr;
	PHY_CHECK(eth->phy_reg_read(eth, ksz8863->addr, ETH_PHY_BMSR_REG_ADDR, &(bmsr.val)) == ESP_OK, "read ID1 failed", err);
	anar_reg_t anar;
	PHY_CHECK(eth->phy_reg_read(eth, ksz8863->addr, ETH_PHY_ANAR_REG_ADDR, &(anar.val)) == ESP_OK, "read ID1 failed", err);
	anlpar_reg_t anlpar;
	PHY_CHECK(eth->phy_reg_read(eth, ksz8863->addr, ETH_PHY_ANLPAR_REG_ADDR, &(anlpar.val)) == ESP_OK, "read ID1 failed", err);
	aner_reg_t aner;
	PHY_CHECK(eth->phy_reg_read(eth, ksz8863->addr, ETH_PHY_ANER_REG_ADDR, &(aner.val)) == ESP_OK, "read ID1 failed", err);
	ESP_LOGI(TAG, "BMSR: %d", bmsr.val);
	ESP_LOGI(TAG, "PHYID: %d", id1.oui_msb);
	ESP_LOGI(TAG, "PHYID: %d", id2.oui_lsb );
	ESP_LOGI(TAG, "PHYID: %d", id2.vendor_model);
	ESP_LOGI(TAG, "anar: %d", anar.val);
	ESP_LOGI(TAG, "anlpar: %d", anlpar.val );
	ESP_LOGI(TAG, "aner: %d", aner.val);
    PHY_CHECK(id1.oui_msb == 0x22 && id2.oui_lsb == 0x5 && id2.vendor_model == 0x3, "wrong PHY chip ID", err);
	lcs_reg_t lcd;
	PHY_CHECK(eth->phy_reg_read(eth, ksz8863->addr, ETH_PHY_LCS_REG_ADDR, &(lcd.val)) == ESP_OK, "read ID1 failed", err);
	ESP_LOGI(TAG, "lcs: %d", lcd.val);
	pscsr_reg_t pscsr;
	PHY_CHECK(eth->phy_reg_read(eth, ksz8863->addr, ETH_PHY_PSCSR_REG_ADDR, &(pscsr.val)) == ESP_OK, "read ID1 failed", err);
	ESP_LOGI(TAG, "pscsr: %d", pscsr.val);
    return ESP_OK;
err:
    return ESP_FAIL;
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
