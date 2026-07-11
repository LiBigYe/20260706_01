#ifndef __NETWORK_PROTOCOL_H
#define __NETWORK_PROTOCOL_H

#include <stdint.h>

#define NET_MIN_DEVICE_ID       1U
#define NET_MAX_DEVICE_ID       9U
#define NET_BROADCAST_MASK      0U
#define NET_VALID_TARGET_MASK   0x01FFU
#define NET_HEADER_BYTES        3U
#define NET_HEADER_SYMBOLS      (NET_HEADER_BYTES * 4U)

static inline uint16_t NET_DeviceMask(uint8_t device_id)
{
    if (device_id < NET_MIN_DEVICE_ID || device_id > NET_MAX_DEVICE_ID) return 0;
    return (uint16_t)(1U << (device_id - 1U));
}

static inline uint8_t NET_IsAddressedTo(uint16_t target_mask, uint8_t device_id)
{
    return (target_mask == NET_BROADCAST_MASK ||
            (target_mask & NET_DeviceMask(device_id)) != 0U) ? 1U : 0U;
}

#endif /* __NETWORK_PROTOCOL_H */
