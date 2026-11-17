// KSZ8863 DS00002335C section 3.3.11, table 3-9. Open-drain MDIO:
// write_data(1) releases the line; callers serialize all management access.
#pragma once
#include <stdint.h>
#include <stdbool.h>
#define SMS_KSZ_SMI_ADDRESS UINT32_C(0x100)
typedef struct {
    void *ctx;
    void (*clock)(void *, int);
    void (*data)(void *, int);
    int (*sample)(void *);
    void (*delay)(void *);
} sms_ksz_smi_bus_t;

static inline void sms_ksz_smi_put(const sms_ksz_smi_bus_t *b, uint32_t value, unsigned bits)
{
    while (bits) {
        --bits;
        b->clock(b->ctx, 0);
        b->data(b->ctx, (int)((value >> bits) & 1));
        b->delay(b->ctx);
        b->clock(b->ctx, 1);
        b->delay(b->ctx);
    }
}

static inline int sms_ksz_smi_get(const sms_ksz_smi_bus_t *b)
{
    b->clock(b->ctx, 0);
    b->delay(b->ctx);
    b->clock(b->ctx, 1);
    b->delay(b->ctx);
    return b->sample(b->ctx) != 0;
}

static inline bool sms_ksz_smi_read(const sms_ksz_smi_bus_t *b, uint8_t reg, uint8_t *out)
{
    if (!out) return false;
    sms_ksz_smi_put(b, UINT32_MAX, 32);
    sms_ksz_smi_put(b, 1, 2); // ST=01
    sms_ksz_smi_put(b, 0, 2); // OP=00, not standard MIIM read OP=10
    sms_ksz_smi_put(b, 0x10U | (reg >> 5), 5);
    sms_ksz_smi_put(b, reg & 31U, 5);
    b->clock(b->ctx, 0);
    b->data(b->ctx, 1); // Hi-Z for both turnaround and data
    (void)sms_ksz_smi_get(b); // Z
    int ack = sms_ksz_smi_get(b); // device must drive 0
    uint32_t value = 0;
    for (unsigned i = 0; i < 16; ++i) value = (value << 1) | (uint32_t)sms_ksz_smi_get(b);
    sms_ksz_smi_put(b, 1, 1); // released idle cycle
    b->clock(b->ctx, 0);
    if (ack || (value & 0xff00U)) return false;
    *out = (uint8_t)value;
    return true;
}

static inline void sms_ksz_smi_write(const sms_ksz_smi_bus_t *b, uint8_t reg, uint8_t value)
{
    sms_ksz_smi_put(b, UINT32_MAX, 32);
    sms_ksz_smi_put(b, 1, 2);
    sms_ksz_smi_put(b, 0, 2);
    sms_ksz_smi_put(b, reg >> 5, 5); // PHYAD[4]=0 selects write
    sms_ksz_smi_put(b, reg & 31U, 5);
    sms_ksz_smi_put(b, 2, 2); // TA=10
    sms_ksz_smi_put(b, value, 16); // upper eight bits zero
    sms_ksz_smi_put(b, 1, 1);
    b->clock(b->ctx, 0);
}

/* v3: diagnostic alternatives; no value is accepted by fixing/forcing bits.
 * Capture 18 response clocks so both turnaround alignments are observable.
 * Linux mdio-bitbang finishes send bits low and checks one turnaround clock;
 * the existing SMS helper explicitly consumes two. Board calibration decides
 * whether any candidate reproduces all independent reference registers. */
#define SMS_KSZ_V3_PROFILES 6U
#define SMS_KSZ_V3_SMI_BASE UINT32_C(0x110)
#define SMS_KSZ_V3_MIIM_BASE UINT32_C(0x200)
#define SMS_KSZ_V3_RAW_BASE UINT32_C(0x300)
enum { SMS_KSZ_SAMPLE_HIGH, SMS_KSZ_SAMPLE_FALL, SMS_KSZ_SAMPLE_LOW };
static inline unsigned sms_ksz_v3_phase(unsigned profile)
{
    return profile < 2 ? SMS_KSZ_SAMPLE_HIGH :
           profile < 4 ? SMS_KSZ_SAMPLE_FALL : SMS_KSZ_SAMPLE_LOW;
}
static inline unsigned sms_ksz_v3_half_us(unsigned profile)
{
    return profile == 0 ? 2U : 8U;
}
static inline unsigned sms_ksz_v3_turnaround(unsigned profile)
{
    return profile == 3 || profile == 5 ? 1U : 2U;
}
static inline const char *sms_ksz_v3_name(unsigned profile)
{
    static const char *const names[SMS_KSZ_V3_PROFILES] = {
        "high-2us-TA2", "high-8us-TA2", "fall-8us-TA2",
        "fall-8us-TA1", "low-8us-TA2", "low-8us-TA1"
    };
    return profile < SMS_KSZ_V3_PROFILES ? names[profile] : "invalid";
}

static inline uint32_t sms_ksz_v3_capture(const sms_ksz_smi_bus_t *b,
    bool smi, uint8_t phy, uint8_t reg, unsigned phase)
{
    sms_ksz_smi_put(b, UINT32_MAX, 32);
    sms_ksz_smi_put(b, 1, 2);
    sms_ksz_smi_put(b, smi ? 0U : 2U, 2);
    sms_ksz_smi_put(b, smi ? (0x10U | (reg >> 5)) : phy, 5);
    sms_ksz_smi_put(b, reg & 31U, 5);
    b->clock(b->ctx, 0);
    b->data(b->ctx, 1);
    uint32_t raw = 0;
    for (unsigned i = 0; i < 18; ++i) {
        b->clock(b->ctx, 0);
        b->delay(b->ctx);
        int bit = 0;
        if (phase == SMS_KSZ_SAMPLE_LOW) bit = b->sample(b->ctx);
        b->clock(b->ctx, 1);
        b->delay(b->ctx);
        if (phase == SMS_KSZ_SAMPLE_HIGH) bit = b->sample(b->ctx);
        b->clock(b->ctx, 0);
        if (phase == SMS_KSZ_SAMPLE_FALL) bit = b->sample(b->ctx);
        raw = (raw << 1) | (uint32_t)(bit != 0);
    }
    sms_ksz_smi_put(b, 1, 1);
    b->clock(b->ctx, 0);
    return raw;
}

static inline bool sms_ksz_v3_decode(uint32_t raw, unsigned profile,
                                    bool smi, uint32_t *value)
{
    if (!value || profile >= SMS_KSZ_V3_PROFILES || raw > 0x3ffffU) return false;
    unsigned shift = sms_ksz_v3_turnaround(profile) == 1 ? 1U : 0U;
    if ((raw >> (16U + shift)) & 1U) return false;
    uint32_t word = (raw >> shift) & 0xffffU;
    if (smi && (word & 0xff00U)) return false;
    *value = word;
    return true;
}

/* Only a fully passing profile may be selected. Prefer the trailing-edge
 * single-TA method, but retain every raw result for review of real hardware. */
static inline int sms_ksz_v3_select(uint32_t passed_mask)
{
    const unsigned preference[] = {3, 2, 1, 0, 4, 5};
    for (unsigned i = 0; i < SMS_KSZ_V3_PROFILES; ++i)
        if (passed_mask & (1U << preference[i])) return (int)preference[i];
    return -1;
}
