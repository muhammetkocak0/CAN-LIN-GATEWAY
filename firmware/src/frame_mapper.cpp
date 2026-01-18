/**
 * @file frame_mapper.cpp
 * @brief Dynamic frame mapping engine implementation
 */

#include "frame_mapper.h"
#include "config.h"
#include <Arduino.h>
#include <string.h>

// External storage functions
extern bool Storage_SaveConfig(const LinConfig_t* linConfig, const CanConfig_t* canConfig,
                               const FrameMapping_t* mappings, uint16_t numMappings);
extern bool Storage_LoadConfig(LinConfig_t* linConfig, CanConfig_t* canConfig,
                               FrameMapping_t* mappings, uint16_t* numMappings);



// External bus functions (implemented in main.cpp)
extern bool LIN_SendFrame(uint8_t frameId, const uint8_t* data, uint8_t len);
extern bool CAN_SendMessage(uint32_t id, const uint8_t* data, uint8_t len);

// ============================================================================
// FRAME MAPPER IMPLEMENTATION
// ============================================================================

FrameMapper::FrameMapper() : mappingCount(0) {
    memset(mappings, 0, sizeof(mappings));
    memset(lastUpdateTime, 0, sizeof(lastUpdateTime));
}

bool FrameMapper::addMapping(const FrameMapping_t* mapping) {
    if (mappingCount >= MAX_MAPPINGS) {
        return false;
    }
    
    // Check if mapping ID already exists
    for (uint16_t i = 0; i < mappingCount; i++) {
        if (mappings[i].mappingId == mapping->mappingId) {
            // Update existing mapping
            memcpy(&mappings[i], mapping, sizeof(FrameMapping_t));
            return true;
        }
    }
    
    // Add new mapping
    memcpy(&mappings[mappingCount], mapping, sizeof(FrameMapping_t));
    mappingCount++;
    
    return true;
}

bool FrameMapper::deleteMapping(uint16_t mappingId) {
    for (uint16_t i = 0; i < mappingCount; i++) {
        if (mappings[i].mappingId == mappingId) {
            // Shift remaining mappings
            for (uint16_t j = i; j < mappingCount - 1; j++) {
                memcpy(&mappings[j], &mappings[j + 1], sizeof(FrameMapping_t));
                lastUpdateTime[j] = lastUpdateTime[j + 1];
            }
            mappingCount--;
            return true;
        }
    }
    return false;
}

void FrameMapper::clearAllMappings() {
    mappingCount = 0;
    memset(mappings, 0, sizeof(mappings));
    memset(lastUpdateTime, 0, sizeof(lastUpdateTime));
}

const FrameMapping_t* FrameMapper::getMappingById(uint16_t mappingId) const {
    for (uint16_t i = 0; i < mappingCount; i++) {
        if (mappings[i].mappingId == mappingId) {
            return &mappings[i];
        }
    }
    return nullptr;
}

uint16_t FrameMapper::getAllMappings(FrameMapping_t* outMappings, uint16_t maxCount) const {
    uint16_t count = min(mappingCount, maxCount);
    memcpy(outMappings, mappings, count * sizeof(FrameMapping_t));
    return count;
}

bool FrameMapper::setMappingEnabled(uint16_t mappingId, bool enabled) {
    for (uint16_t i = 0; i < mappingCount; i++) {
        if (mappings[i].mappingId == mappingId) {
            mappings[i].enabled = enabled ? 1 : 0;
            return true;
        }
    }
    return false;
}

bool FrameMapper::saveToFlash() {
    return Storage_SaveConfig(&gateway.linConfig, &gateway.canConfig, 
                             mappings, mappingCount);
}

bool FrameMapper::loadFromFlash() {
    uint16_t loadedCount = 0;
    bool success = Storage_LoadConfig(&gateway.linConfig, &gateway.canConfig,
                                     mappings, &loadedCount);
    if (success) {
        mappingCount = loadedCount;
        memset(lastUpdateTime, 0, sizeof(lastUpdateTime));
    }
    return success;
}

int16_t FrameMapper::findMappingBySource(uint8_t sourceType, uint32_t sourceId) const {
    for (uint16_t i = 0; i < mappingCount; i++) {
        if (mappings[i].enabled && 
            mappings[i].sourceType == sourceType && 
            mappings[i].sourceId == sourceId) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief Extract signal value from byte array
 */
uint64_t FrameMapper::extractSignal(const uint8_t* srcData, uint8_t bitStart, 
                                   uint8_t bitLen, uint8_t byteOrder) const {
    uint64_t value = 0;
    
    if (byteOrder == BYTE_ORDER_LITTLE) {
        // Little Endian (Intel format)
        uint8_t startByte = bitStart / 8;
        uint8_t startBit = bitStart % 8;
        
        for (uint8_t i = 0; i < bitLen; i++) {
            uint8_t byteIdx = (startBit + i) / 8 + startByte;
            uint8_t bitIdx = (startBit + i) % 8;
            
            if (srcData[byteIdx] & (1 << bitIdx)) {
                value |= (uint64_t)1 << i;
            }
        }
    } else {
        // Big Endian (Motorola format)
        for (uint8_t i = 0; i < bitLen; i++) {
            uint8_t bitPos = bitStart + i;
            uint8_t byteIdx = bitPos / 8;
            uint8_t bitIdx = 7 - (bitPos % 8);
            
            if (srcData[byteIdx] & (1 << bitIdx)) {
                value |= (uint64_t)1 << (bitLen - 1 - i);
            }
        }
    }
    
    return value;
}

/**
 * @brief Pack signal value into byte array
 */
void FrameMapper::packSignal(uint8_t* destData, uint64_t value, uint8_t bitStart, 
                            uint8_t bitLen, uint8_t byteOrder) const {
    if (byteOrder == BYTE_ORDER_LITTLE) {
        // Little Endian
        uint8_t startByte = bitStart / 8;
        uint8_t startBit = bitStart % 8;
        
        for (uint8_t i = 0; i < bitLen; i++) {
            uint8_t byteIdx = (startBit + i) / 8 + startByte;
            uint8_t bitIdx = (startBit + i) % 8;
            
            if (value & ((uint64_t)1 << i)) {
                destData[byteIdx] |= (1 << bitIdx);
            } else {
                destData[byteIdx] &= ~(1 << bitIdx);
            }
        }
    } else {
        // Big Endian
        for (uint8_t i = 0; i < bitLen; i++) {
            uint8_t bitPos = bitStart + i;
            uint8_t byteIdx = bitPos / 8;
            uint8_t bitIdx = 7 - (bitPos % 8);
            
            if (value & ((uint64_t)1 << (bitLen - 1 - i))) {
                destData[byteIdx] |= (1 << bitIdx);
            } else {
                destData[byteIdx] &= ~(1 << bitIdx);
            }
        }
    }
}

float FrameMapper::applyScaling(uint64_t value, float scale, float offset) const {
    return (float)value * scale + offset;
}

uint64_t FrameMapper::reverseScaling(float value, float scale, float offset) const {
    return (uint64_t)((value - offset) / scale);
}

bool FrameMapper::checkUpdateRate(uint16_t mappingIndex) {
    if (mappings[mappingIndex].updateRateMs == 0) {
        return true;  // No rate limit
    }
    
    uint32_t now = millis();
    if (now - lastUpdateTime[mappingIndex] >= mappings[mappingIndex].updateRateMs) {
        lastUpdateTime[mappingIndex] = now;
        return true;
    }
    
    return false;
}

bool FrameMapper::generateCanMessage(const FrameMapping_t* mapping, const uint8_t* srcData) {
    uint8_t destData[8] = {0};
    
    // Process all signals
    for (uint8_t i = 0; i < mapping->numSignals; i++) {
        const SignalMapping_t* sig = &mapping->signals[i];
        
        // Extract from source
        uint64_t rawValue = extractSignal(srcData, sig->srcBitStart, sig->srcBitLen, sig->byteOrder);
        
        // Apply scaling
        float scaledValue = applyScaling(rawValue, sig->scale, sig->offset);
        
        // Reverse scaling to destination range
        uint64_t destValue = reverseScaling(scaledValue, 1.0, 0.0);  // TODO: dest scaling
        
        // Pack to destination
        packSignal(destData, destValue, sig->destBitStart, sig->destBitLen, sig->byteOrder);
    }
    
    // Send CAN message
    return CAN_SendMessage(mapping->destId, destData, mapping->destLen);
}

bool FrameMapper::generateLinFrame(const FrameMapping_t* mapping, const uint8_t* srcData) {
    uint8_t destData[8] = {0};
    
    // Process all signals (similar to CAN generation)
    for (uint8_t i = 0; i < mapping->numSignals; i++) {
        const SignalMapping_t* sig = &mapping->signals[i];
        
        uint64_t rawValue = extractSignal(srcData, sig->srcBitStart, sig->srcBitLen, sig->byteOrder);
        float scaledValue = applyScaling(rawValue, sig->scale, sig->offset);
        uint64_t destValue = reverseScaling(scaledValue, 1.0, 0.0);
        
        packSignal(destData, destValue, sig->destBitStart, sig->destBitLen, sig->byteOrder);
    }
    
    // Send LIN frame
    return LIN_SendFrame((uint8_t)mapping->destId, destData, mapping->destLen);
}

uint8_t FrameMapper::processLinFrame(uint8_t frameId, const uint8_t* data, uint8_t len) {
    uint8_t count = 0;
    
    // Find all mappings with this LIN frame as source
    for (uint16_t i = 0; i < mappingCount; i++) {
        if (mappings[i].enabled && 
            mappings[i].sourceType == BUS_TYPE_LIN &&
            mappings[i].sourceId == frameId) {
            
            // Check update rate
            if (!checkUpdateRate(i)) {
                continue;
            }
            
            // Generate destination frame
            if (mappings[i].destType == BUS_TYPE_CAN) {
                if (generateCanMessage(&mappings[i], data)) {
                    count++;
                } else {
                    gateway.stats.mappingErrors++;
                }
            } else if (mappings[i].destType == BUS_TYPE_LIN) {
                if (generateLinFrame(&mappings[i], data)) {
                    count++;
                } else {
                    gateway.stats.mappingErrors++;
                }
            }
        }
    }
    
    return count;
}

uint8_t FrameMapper::processCanMessage(uint32_t msgId, const uint8_t* data, uint8_t len) {
    uint8_t count = 0;
    
    // Find all mappings with this CAN message as source
    for (uint16_t i = 0; i < mappingCount; i++) {
        if (mappings[i].enabled && 
            mappings[i].sourceType == BUS_TYPE_CAN &&
            mappings[i].sourceId == msgId) {
            
            // Check update rate
            if (!checkUpdateRate(i)) {
                continue;
            }
            
            // Generate destination frame
            if (mappings[i].destType == BUS_TYPE_LIN) {
                if (generateLinFrame(&mappings[i], data)) {
                    count++;
                } else {
                    gateway.stats.mappingErrors++;
                }
            } else if (mappings[i].destType == BUS_TYPE_CAN) {
                if (generateCanMessage(&mappings[i], data)) {
                    count++;
                } else {
                    gateway.stats.mappingErrors++;
                }
            }
        }
    }
    
    return count;
}

// Global instance
FrameMapper frameMapper;