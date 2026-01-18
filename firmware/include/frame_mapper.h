/**
 * @file frame_mapper.h
 * @brief Dynamic frame mapping engine for CAN-LIN gateway
 * 
 * Handles dynamic mapping between LIN frames and CAN messages
 * based on runtime configuration
 */

#ifndef FRAME_MAPPER_H
#define FRAME_MAPPER_H

#include "protocol.h"
#include <stdint.h>

#define MAX_MAPPINGS 64  // Maximum number of frame mappings

/**
 * @brief Frame Mapper class
 * 
 * Manages dynamic mapping between LIN and CAN frames
 */
class FrameMapper {
public:
    FrameMapper();
    
    /**
     * @brief Add a new frame mapping
     * @param mapping Pointer to frame mapping structure
     * @return true if successful, false if mapping table full
     */
    bool addMapping(const FrameMapping_t* mapping);
    
    /**
     * @brief Delete a frame mapping
     * @param mappingId Mapping ID to delete
     * @return true if successful, false if not found
     */
    bool deleteMapping(uint16_t mappingId);
    
    /**
     * @brief Delete all mappings
     */
    void clearAllMappings();
    
    /**
     * @brief Get number of active mappings
     * @return Number of mappings
     */
    uint16_t getMappingCount() const { return mappingCount; }
    
    /**
     * @brief Process received LIN frame
     * @param frameId LIN frame ID
     * @param data Pointer to frame data
     * @param len Frame length
     * @return Number of CAN messages generated
     */
    uint8_t processLinFrame(uint8_t frameId, const uint8_t* data, uint8_t len);
    
    /**
     * @brief Process received CAN message
     * @param msgId CAN message ID
     * @param data Pointer to message data
     * @param len Message length
     * @return Number of LIN frames generated
     */
    uint8_t processCanMessage(uint32_t msgId, const uint8_t* data, uint8_t len);
    
    /**
     * @brief Get mapping by ID
     * @param mappingId Mapping ID
     * @return Pointer to mapping or nullptr if not found
     */
    const FrameMapping_t* getMappingById(uint16_t mappingId) const;
    
    /**
     * @brief Get all mappings
     * @param mappings Buffer to store mappings
     * @param maxCount Maximum number of mappings to retrieve
     * @return Actual number of mappings retrieved
     */
    uint16_t getAllMappings(FrameMapping_t* mappings, uint16_t maxCount) const;
    
    /**
     * @brief Enable/disable a mapping
     * @param mappingId Mapping ID
     * @param enabled true to enable, false to disable
     * @return true if successful
     */
    bool setMappingEnabled(uint16_t mappingId, bool enabled);
    
    /**
     * @brief Save all mappings to Flash
     * @return true if successful
     */
    bool saveToFlash();
    
    /**
     * @brief Load mappings from Flash
     * @return true if successful
     */
    bool loadFromFlash();
    
private:
    // Mapping storage
    FrameMapping_t mappings[MAX_MAPPINGS];
    uint16_t mappingCount;
    uint32_t lastUpdateTime[MAX_MAPPINGS];  // For rate limiting
    
    /**
     * @brief Find mapping index by source
     * @param sourceType Source bus type (LIN/CAN)
     * @param sourceId Source frame/message ID
     * @return Mapping index or -1 if not found
     */
    int16_t findMappingBySource(uint8_t sourceType, uint32_t sourceId) const;
    
    /**
     * @brief Extract signal from source data
     * @param srcData Source data buffer
     * @param bitStart Start bit position
     * @param bitLen Bit length
     * @param byteOrder Byte order
     * @return Extracted value as uint64_t
     */
    uint64_t extractSignal(const uint8_t* srcData, uint8_t bitStart, 
                          uint8_t bitLen, uint8_t byteOrder) const;
    
    /**
     * @brief Pack signal into destination data
     * @param destData Destination data buffer
     * @param value Value to pack
     * @param bitStart Start bit position
     * @param bitLen Bit length
     * @param byteOrder Byte order
     */
    void packSignal(uint8_t* destData, uint64_t value, uint8_t bitStart, 
                   uint8_t bitLen, uint8_t byteOrder) const;
    
    /**
     * @brief Apply scaling and offset to signal value
     * @param value Raw value
     * @param scale Scaling factor
     * @param offset Offset
     * @return Scaled value
     */
    float applyScaling(uint64_t value, float scale, float offset) const;
    
    /**
     * @brief Reverse scaling (for opposite direction)
     * @param value Scaled value
     * @param scale Scaling factor
     * @param offset Offset
     * @return Raw value
     */
    uint64_t reverseScaling(float value, float scale, float offset) const;
    
    /**
     * @brief Check if mapping should update based on rate limit
     * @param mappingIndex Mapping index
     * @return true if should update
     */
    bool checkUpdateRate(uint16_t mappingIndex);
    
    /**
     * @brief Generate CAN message from mapping
     * @param mapping Pointer to mapping
     * @param srcData Source LIN data
     * @return true if message generated and queued
     */
    bool generateCanMessage(const FrameMapping_t* mapping, const uint8_t* srcData);
    
    /**
     * @brief Generate LIN frame from mapping
     * @param mapping Pointer to mapping
     * @param srcData Source CAN data
     * @return true if frame generated and queued
     */
    bool generateLinFrame(const FrameMapping_t* mapping, const uint8_t* srcData);
};

// Global frame mapper instance
extern FrameMapper frameMapper;

#endif // FRAME_MAPPER_H