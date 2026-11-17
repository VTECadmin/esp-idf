// Shared SMS host-link policy: ESP32 MAC and KSZ port 3 must agree.
#pragma once
#define SMS_KSZ_HOST_CONTROL4 0x70U /* H-10half: half duplex, 10M; retain flow control */
#define SMS_KSZ_HOST_SPEED ETH_SPEED_10M
#define SMS_KSZ_HOST_DUPLEX ETH_DUPLEX_HALF
#define SMS_KSZ_HOST_SPEED_MBPS 10U
#define SMS_KSZ_IO_DRIVE_MASK 0x40U /* G0E bit6: 16 mA */
#define SMS_KSZ_RMII_INVERT_MASK 0x80U /* P36 bit7: inverted RMII clock */
#define SMS_KSZ_RMII_INVERT_VALUE 0x00U /* H-10half: original/non-inverted clock */
