/**
 * @file config.h
 * @brief Hardware and system configuration
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
// Ensure these types are known. If they are in "protocol.h" or "types.h", include that here.
// Assuming LinConfig_t, CanConfig_t, GatewayStats_t are defined in a common header (e.g., types.h).
// If they are not defined yet, you might need to include that header or forward declare them.

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// LIN Interface (UART2)
#define LIN_UART_TX         PA2
#define LIN_UART_RX         PA3
#define LIN_UART_INSTANCE   USART2

// CAN Interface (CAN1)
#define CAN_RX_PIN          PB8
#define CAN_TX_PIN          PB9

// Status LED
#define LED_STATUS          LED_BUILTIN

// ============================================================================
// SYSTEM CONFIGURATION
// ============================================================================

// Default bus configurations
#define DEFAULT_LIN_BAUD    19200
#define DEFAULT_CAN_BAUD    500000

// Maximum mappings
#define MAX_MAPPINGS        64

// Flash configuration storage
#define CONFIG_FLASH_SECTOR         FLASH_SECTOR_11
#define CONFIG_FLASH_BASE_ADDRESS   0x080E0000  // Last sector
#define CONFIG_FLASH_SIZE           (128 * 1024) // 128KB

// Protocol configuration
#define PROTOCOL_TIMEOUT_MS     1000
#define PROTOCOL_MAX_PAYLOAD    512

// Queue sizes
#define LIN_TX_QUEUE_SIZE   16
#define LIN_RX_QUEUE_SIZE   16
#define CAN_TX_QUEUE_SIZE   32
#define CAN_RX_QUEUE_SIZE   32

// Timing
#define GATEWAY_LOOP_INTERVAL_MS    1
#define STATUS_UPDATE_INTERVAL_MS   1000

// Debug
#define DEBUG_SERIAL        1
#define DEBUG_VERBOSE       0

// ============================================================================
// VERSION INFORMATION
// ============================================================================

#define FIRMWARE_VERSION_MAJOR  2
#define FIRMWARE_VERSION_MINOR  0
#define FIRMWARE_VERSION_PATCH  0
#define FIRMWARE_BUILD_DATE     __DATE__
#define FIRMWARE_BUILD_TIME     __TIME__
typedef struct {
    bool running;
    LinConfig_t linConfig;
    CanConfig_t canConfig;
    GatewayStats_t stats;
    uint32_t startTime;
} GatewayState_t;

// Declare the external instance so all files know 'gateway' exists
extern GatewayState_t gateway;
#endif // CONFIG_H