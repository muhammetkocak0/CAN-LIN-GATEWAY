/**
 * @file protocol.cpp
 * @brief Serial configuration protocol implementation
 */

#include "protocol.h"
#include "config.h"       // Defines 'extern GatewayState_t gateway;'
#include <Arduino.h>
#include "frame_mapper.h" // Fixes incomplete type error for frameMapper

// External functions from storage.cpp
extern bool Storage_SaveConfig(const LinConfig_t* linConfig, const CanConfig_t* canConfig,
                               const FrameMapping_t* mappings, uint16_t numMappings);
extern bool Storage_LoadConfig(LinConfig_t* linConfig, CanConfig_t* canConfig,
                               FrameMapping_t* mappings, uint16_t* numMappings);

// ============================================================================
// EXTERNAL VARIABLES
// ============================================================================

// NOTE: 'gateway' is already declared in config.h. 
// DO NOT re-declare it here as 'extern struct { ... } gateway'.

// External frame mapper (if not declared in frame_mapper.h)
extern FrameMapper frameMapper;

// ============================================================================
// PROTOCOL CLASS IMPLEMENTATION
// ============================================================================

Protocol::Protocol() : rxState(RX_WAIT_SOF), rxIndex(0) {
    memset(&rxBuffer, 0, sizeof(rxBuffer));
}

/**
 * @brief Calculate CRC16-CCITT checksum
 */
uint16_t Protocol::calculateCRC16(const uint8_t* data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

/**
 * @brief Verify CRC16 of received message
 */
bool Protocol::verifyCRC16(const ProtocolMessage_t* msg) {
    // Calculate CRC of: SOF + CMD + LENGTH + PAYLOAD
    uint8_t buffer[PROTOCOL_MAX_PAYLOAD + 4];
    uint16_t idx = 0;
    
    buffer[idx++] = msg->sof;
    buffer[idx++] = msg->cmd;
    buffer[idx++] = msg->length & 0xFF;
    buffer[idx++] = (msg->length >> 8) & 0xFF;
    
    for (uint16_t i = 0; i < msg->length; i++) {
        buffer[idx++] = msg->payload[i];
    }
    
    uint16_t calculated = calculateCRC16(buffer, idx);
    
    return (calculated == msg->checksum);
}

/**
 * @brief Receive and parse protocol message
 */
bool Protocol::receiveMessage(ProtocolMessage_t* msg) {
    while (Serial.available()) {
        uint8_t byte = Serial.read();
        
        switch (rxState) {
            case RX_WAIT_SOF:
                if (byte == PROTOCOL_SOF) {
                    rxBuffer.sof = byte;
                    rxState = RX_WAIT_CMD;
                    rxIndex = 0;
                }
                break;
                
            case RX_WAIT_CMD:
                rxBuffer.cmd = byte;
                rxState = RX_WAIT_LENGTH_LOW;
                break;
                
            case RX_WAIT_LENGTH_LOW:
                rxBuffer.length = byte;
                rxState = RX_WAIT_LENGTH_HIGH;
                break;
                
            case RX_WAIT_LENGTH_HIGH:
                rxBuffer.length |= (uint16_t)byte << 8;
                
                if (rxBuffer.length > PROTOCOL_MAX_PAYLOAD) {
                    rxState = RX_WAIT_SOF;  // Invalid length
                } else if (rxBuffer.length == 0) {
                    rxState = RX_WAIT_CRC_LOW;  // No payload
                } else {
                    rxState = RX_WAIT_PAYLOAD;
                    rxIndex = 0;
                }
                break;
                
            case RX_WAIT_PAYLOAD:
                rxBuffer.payload[rxIndex++] = byte;
                if (rxIndex >= rxBuffer.length) {
                    rxState = RX_WAIT_CRC_LOW;
                }
                break;
                
            case RX_WAIT_CRC_LOW:
                rxBuffer.checksum = byte;
                rxState = RX_WAIT_CRC_HIGH;
                break;
                
            case RX_WAIT_CRC_HIGH:
                rxBuffer.checksum |= (uint16_t)byte << 8;
                rxState = RX_WAIT_EOF;
                break;
                
            case RX_WAIT_EOF:
                if (byte == PROTOCOL_EOF) {
                    rxBuffer.eof = byte;
                    
                    // Verify checksum
                    if (verifyCRC16(&rxBuffer)) {
                        memcpy(msg, &rxBuffer, sizeof(ProtocolMessage_t));
                        rxState = RX_WAIT_SOF;
                        return true;
                    }
                }
                rxState = RX_WAIT_SOF;
                break;
        }
    }
    
    return false;
}

/**
 * @brief Send response message
 */
void Protocol::sendResponse(uint8_t cmd, uint8_t status, const uint8_t* data, uint16_t len) {
    uint8_t buffer[PROTOCOL_MAX_PAYLOAD + 16];
    uint16_t idx = 0;
    
    // Build message
    buffer[idx++] = PROTOCOL_SOF;
    buffer[idx++] = cmd;
    buffer[idx++] = (len + 1) & 0xFF;  // +1 for status byte
    buffer[idx++] = ((len + 1) >> 8) & 0xFF;
    buffer[idx++] = status;
    
    if (data && len > 0) {
        memcpy(&buffer[idx], data, len);
        idx += len;
    }
    
    // Calculate CRC
    uint16_t crc = calculateCRC16(buffer, idx);
    buffer[idx++] = crc & 0xFF;
    buffer[idx++] = (crc >> 8) & 0xFF;
    buffer[idx++] = PROTOCOL_EOF;
    
    // Send
    Serial.write(buffer, idx);
    Serial.flush();
}

/**
 * @brief Send status update
 */
void Protocol::sendStatus(const GatewayStatus_t* status) {
    sendResponse(CMD_GET_STATUS, RESP_OK, (uint8_t*)status, sizeof(GatewayStatus_t));
}

/**
 * @brief Send statistics
 */
void Protocol::sendStatistics(const GatewayStats_t* stats) {
    sendResponse(CMD_GET_STATISTICS, RESP_OK, (uint8_t*)stats, sizeof(GatewayStats_t));
}

/**
 * @brief Process received command
 */
void Protocol::processCommand(ProtocolMessage_t* msg) {
    #ifdef DEBUG_VERBOSE
    Serial.print("[PROTO] CMD: 0x");
    Serial.print(msg->cmd, HEX);
    Serial.print(", LEN: ");
    Serial.println(msg->length);
    #endif
    
    switch (msg->cmd) {
        case CMD_SET_LIN_CONFIG:
            if (msg->length >= sizeof(LinConfig_t)) {
                LinConfig_t* config = (LinConfig_t*)msg->payload;
                memcpy(&gateway.linConfig, config, sizeof(LinConfig_t));
                sendResponse(CMD_SET_LIN_CONFIG, RESP_OK, nullptr, 0);
                
                #ifdef DEBUG_SERIAL
                Serial.print("[CFG] LIN Baud: ");
                Serial.println(config->baudRate);
                #endif
            } else {
                sendResponse(CMD_SET_LIN_CONFIG, RESP_INVALID_PARAM, nullptr, 0);
            }
            break;
            
        case CMD_SET_CAN_CONFIG:
            if (msg->length >= sizeof(CanConfig_t)) {
                CanConfig_t* config = (CanConfig_t*)msg->payload;
                memcpy(&gateway.canConfig, config, sizeof(CanConfig_t));
                sendResponse(CMD_SET_CAN_CONFIG, RESP_OK, nullptr, 0);
                
                #ifdef DEBUG_SERIAL
                Serial.print("[CFG] CAN Baud: ");
                Serial.println(config->baudRate);
                #endif
            } else {
                sendResponse(CMD_SET_CAN_CONFIG, RESP_INVALID_PARAM, nullptr, 0);
            }
            break;
            
        case CMD_ADD_MAPPING:
            if (msg->length >= sizeof(FrameMapping_t)) {
                FrameMapping_t* mapping = (FrameMapping_t*)msg->payload;
                if (frameMapper.addMapping(mapping)) {
                    sendResponse(CMD_ADD_MAPPING, RESP_OK, nullptr, 0);
                    
                    #ifdef DEBUG_SERIAL
                    Serial.print("[MAP] Added: ");
                    Serial.print(mapping->sourceType == BUS_TYPE_LIN ? "LIN" : "CAN");
                    Serial.print(" 0x");
                    Serial.print(mapping->sourceId, HEX);
                    Serial.print(" -> ");
                    Serial.print(mapping->destType == BUS_TYPE_LIN ? "LIN" : "CAN");
                    Serial.print(" 0x");
                    Serial.println(mapping->destId, HEX);
                    #endif
                } else {
                    sendResponse(CMD_ADD_MAPPING, RESP_ERROR, nullptr, 0);
                }
            } else {
                sendResponse(CMD_ADD_MAPPING, RESP_INVALID_PARAM, nullptr, 0);
            }
            break;
            
        case CMD_DELETE_MAPPING:
            if (msg->length >= sizeof(uint16_t)) {
                uint16_t mappingId = *(uint16_t*)msg->payload;
                if (frameMapper.deleteMapping(mappingId)) {
                    sendResponse(CMD_DELETE_MAPPING, RESP_OK, nullptr, 0);
                } else {
                    sendResponse(CMD_DELETE_MAPPING, RESP_ERROR, nullptr, 0);
                }
            } else {
                sendResponse(CMD_DELETE_MAPPING, RESP_INVALID_PARAM, nullptr, 0);
            }
            break;
            
        case CMD_SAVE_CONFIG:
            if (frameMapper.saveToFlash()) {
                sendResponse(CMD_SAVE_CONFIG, RESP_OK, nullptr, 0);
                #ifdef DEBUG_SERIAL
                Serial.println("[FLASH] Configuration saved");
                #endif
            } else {
                sendResponse(CMD_SAVE_CONFIG, RESP_FLASH_ERROR, nullptr, 0);
            }
            break;
            
        case CMD_LOAD_CONFIG:
            if (frameMapper.loadFromFlash()) {
                sendResponse(CMD_LOAD_CONFIG, RESP_OK, nullptr, 0);
                #ifdef DEBUG_SERIAL
                Serial.println("[FLASH] Configuration loaded");
                #endif
            } else {
                sendResponse(CMD_LOAD_CONFIG, RESP_FLASH_ERROR, nullptr, 0);
            }
            break;
            
        case CMD_GET_STATUS: {
            GatewayStatus_t status;
            status.running = gateway.running;
            status.linStatus = 1;  // TODO: Real status
            status.canStatus = 1;
            status.numMappings = frameMapper.getMappingCount();
            status.uptime = (millis() - gateway.startTime) / 1000;
            status.flashUsage = 0;  // TODO: Calculate
            
            sendStatus(&status);
            break;
        }
            
        case CMD_START_GATEWAY:
            gateway.running = true;
            sendResponse(CMD_START_GATEWAY, RESP_OK, nullptr, 0);
            #ifdef DEBUG_SERIAL
            Serial.println("[GW] Started");
            #endif
            break;
            
        case CMD_STOP_GATEWAY:
            gateway.running = false;
            sendResponse(CMD_STOP_GATEWAY, RESP_OK, nullptr, 0);
            #ifdef DEBUG_SERIAL
            Serial.println("[GW] Stopped");
            #endif
            break;
            
        case CMD_GET_STATISTICS:
            sendStatistics(&gateway.stats);
            break;
            
        case CMD_CLEAR_MAPPINGS:
            frameMapper.clearAllMappings();
            sendResponse(CMD_CLEAR_MAPPINGS, RESP_OK, nullptr, 0);
            #ifdef DEBUG_SERIAL
            Serial.println("[MAP] All cleared");
            #endif
            break;
            
        case CMD_RESET_GATEWAY:
            sendResponse(CMD_RESET_GATEWAY, RESP_OK, nullptr, 0);
            delay(100);
            NVIC_SystemReset();
            break;
            
        default:
            sendResponse(msg->cmd, RESP_INVALID_CMD, nullptr, 0);
            break;
    }
}