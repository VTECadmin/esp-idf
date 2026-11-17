// SMS diagnostic extension for the supported ESP-IDF fork.
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t submitted_frames, completed_frames, completed_desc;
    uint32_t error_frames, error_desc, pending_desc, abandoned_desc;
    uint32_t underflow, excessive_deferral, excessive_collision, late_collision;
    uint32_t no_carrier, loss_carrier, flushed, jabber, checksum_error;
    uint32_t error_status_or, last_error_status;
} esp_eth_tx_diag_stats_t;

// Called only with CPU serialization held. OWN=0 publishes DMA status. A slot
// is counted once per successful submission, before its status is overwritten.
// TDES0 layout: ESP32 TRM, EMAC enhanced transmit descriptor 0.
static inline void esp_eth_tx_diag_reap(esp_eth_tx_diag_stats_t *s,
                                      bool *pending, uint32_t status)
{
    if (!*pending || (status & (UINT32_C(1) << 31))) return;
    *pending = false;
    ++s->completed_desc;
    if (status & (UINT32_C(1) << 29)) ++s->completed_frames;
    if (!(status & (UINT32_C(1) << 15))) return;
    ++s->error_desc;
    if (status & (UINT32_C(1) << 29)) ++s->error_frames;
    s->last_error_status = status;
    s->error_status_or |= status & UINT32_C(0x1ffff);
    s->underflow += !!(status & (UINT32_C(1) << 1));
    s->excessive_deferral += !!(status & (UINT32_C(1) << 2));
    s->excessive_collision += !!(status & (UINT32_C(1) << 8));
    s->late_collision += !!(status & (UINT32_C(1) << 9));
    s->no_carrier += !!(status & (UINT32_C(1) << 10));
    s->loss_carrier += !!(status & (UINT32_C(1) << 11));
    s->flushed += !!(status & (UINT32_C(1) << 13));
    s->jabber += !!(status & (UINT32_C(1) << 14));
    s->checksum_error += !!(status & ((UINT32_C(1) << 12) | (UINT32_C(1) << 16)));
}
