/**
 * @file protocol.h
 * @brief Serial configuration protocol definitions
 * 
 * Binary protocol for configuring CAN-LIN gateway via USB serial
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

// Protocol constants
#define PROTOCOL_SOF          0xAA
#define PROTOCOL_EOF          0x55
#define PROTOCOL_MAX_PAYLOAD  512
#define PROTOCOL_TIMEOUT_MS   1000

// Command codes
#define CMD_SET_LIN_CONFIG    0x01
#define CMD_SET_CAN_CONFIG    0x02
#define CMD_ADD_MAPPING       0x03
#define CMD_DELETE_MAPPING    0x04
#define CMD_SAVE_CONFIG       0x05
#define CMD_LOAD_CONFIG       0x06
#define CMD_GET_STATUS        0x07
#define CMD_START_GATEWAY     0x08
#define CMD_STOP_GATEWAY      0x09
#define CMD_GET_STATISTICS    0x0A
#define CMD_CLEAR_MAPPINGS    0x0B
#define CMD_RESET_GATEWAY     0xFF

// Response codes
#define RESP_OK               0x00
#define RESP_ERROR            0x01
#define RESP_INVALID_CMD      0x02
#define RESP_INVALID_PARAM    0x03
#define RESP_FLASH_ERROR      0x04
#define RESP_BUSY             0x05

// Bus types
#define BUS_TYPE_LIN          0x00
#define BUS_TYPE_CAN          0x01

// Byte order
#define BYTE_ORDER_LITTLE     0x00
#define BYTE_ORDER_BIG        0x01

// Protocol message structure
typedef struct __attribute__((packed)) {
    uint8_t  sof;           // Start of frame (0xAA)
    uint8_t  cmd;           // Command code
    uint16_t length;        // Payload length
    uint8_t  payload[PROTOCOL_MAX_PAYLOAD];
    uint16_t checksum;      // CRC16
    uint8_t  eof;           // End of frame (0x55)
} ProtocolMessage_t;

// LIN Configuration
typedef struct __attribute__((packed)) {
    uint32_t baudRate;      // LIN baud rate (e.g., 19200)
    uint8_t  mode;          // 0=Slave, 1=Master
    uint8_t  breakLength;   // Break length in bits (13-16)
} LinConfig_t;

// CAN Configuration
typedef struct __attribute__((packed)) {
    uint32_t baudRate;      // CAN baud rate (e.g., 500000)
    uint8_t  mode;          // 0=Normal, 1=Loopback, 2=Silent
    uint8_t  numFilters;    // Number of acceptance filters
    struct {
        uint32_t id;        // Filter ID
        uint32_t mask;      // Filter mask
    } filters[8];
} CanConfig_t;

// Signal mapping definition
typedef struct __attribute__((packed)) {
    uint8_t  srcBitStart;   // Source bit position (0-63)
    uint8_t  srcBitLen;     // Source bit length (1-32)
    uint8_t  destBitStart;  // Destination bit position (0-63)
    uint8_t  destBitLen;    // Destination bit length (1-32)
    float    scale;         // Scaling factor
    float    offset;        // Offset value
    uint8_t  byteOrder;     // 0=Little Endian, 1=Big Endian
    uint8_t  reserved;      // Padding
} SignalMapping_t;

// Frame mapping definition
typedef struct __attribute__((packed)) {
    // Mapping ID
    uint16_t mappingId;     // Unique mapping ID
    
    // Source
    uint8_t  sourceType;    // 0=LIN, 1=CAN
    uint32_t sourceId;      // LIN Frame ID or CAN Message ID
    uint8_t  sourceLen;     // Source frame length (bytes)
    
    // Destination
    uint8_t  destType;      // 0=LIN, 1=CAN
    uint32_t destId;        // LIN Frame ID or CAN Message ID
    uint8_t  destLen;       // Destination frame length (bytes)
    
    // Signal mappings
    uint8_t  numSignals;    // Number of signal mappings (0-16)
    SignalMapping_t signals[16];
    
    // Options
    uint16_t updateRateMs;  // Minimum update rate in ms (0=immediate)
    uint8_t  enabled;       // 1=enabled, 0=disabled
    uint8_t  direction;     // 0=Unidirectional, 1=Bidirectional
    
    // Reserved for future use
    uint8_t  reserved[8];
} FrameMapping_t;

// Gateway status
typedef struct __attribute__((packed)) {
    uint8_t  running;       // 1=running, 0=stopped
    uint8_t  linStatus;     // LIN bus status
    uint8_t  canStatus;     // CAN bus status
    uint16_t numMappings;   // Number of active mappings
    uint32_t uptime;        // Uptime in seconds
    uint16_t flashUsage;    // Flash usage percentage
} GatewayStatus_t;

// Gateway statistics
typedef struct __attribute__((packed)) {
    uint32_t linFramesRx;   // LIN frames received
    uint32_t linFramesTx;   // LIN frames transmitted
    uint32_t canFramesRx;   // CAN frames received
    uint32_t canFramesTx;   // CAN frames transmitted
    uint32_t linErrors;     // LIN errors
    uint32_t canErrors;     // CAN errors
    uint32_t mappingErrors; // Mapping errors
    uint32_t checksumErrors;// Checksum errors
} GatewayStats_t;

// Function prototypes
class Protocol {
public:
    Protocol();
    
    // Receive and parse message
    bool receiveMessage(ProtocolMessage_t* msg);
    
    // Send response
    void sendResponse(uint8_t cmd, uint8_t status, const uint8_t* data, uint16_t len);
    
    // Send status update
    void sendStatus(const GatewayStatus_t* status);
    
    // Send statistics
    void sendStatistics(const GatewayStats_t* stats);
    
    // Process received command
    void processCommand(ProtocolMessage_t* msg);
    
private:
    // Calculate CRC16
    uint16_t calculateCRC16(const uint8_t* data, uint16_t length);
    
    // Verify CRC16
    bool verifyCRC16(const ProtocolMessage_t* msg);
    
    // State machine for receiving
    enum RxState {
        RX_WAIT_SOF,
        RX_WAIT_CMD,
        RX_WAIT_LENGTH_LOW,
        RX_WAIT_LENGTH_HIGH,
        RX_WAIT_PAYLOAD,
        RX_WAIT_CRC_LOW,
        RX_WAIT_CRC_HIGH,
        RX_WAIT_EOF
    };
    
    RxState rxState;
    uint16_t rxIndex;
    ProtocolMessage_t rxBuffer;
};

#endif // PROTOCOL_H