/**
 * @file main.cpp
 * @brief CAN-LIN Passive Gateway for BLDC Motor Data Acquisition
 * @author STM32 Gateway
 * @date 2026
 * 
 * Purpose: Passive monitoring of LIN bus communication between Vehicle ECU
 *          and BLDC motor. Forwards motor telemetry to CAN for SCADAS XS.
 * 
 * Hardware: STM32F446RE Nucleo Board
 * LIN: UART2 (PA2-TX, PA3-RX) with TJA1020/TJA1021 transceiver - RX ONLY
 * CAN: CAN1 (PB8-RX, PB9-TX) with TJA1050 transceiver
 * Debug: UART via USB (Serial)
 * 
 * Operation: PASSIVE SNIFFER - Does NOT control BLDC motor
 */

#include <Arduino.h>
#include <HardwareSerial.h>

// ============================================================================
// CONFIGURATION
// ============================================================================
#define LIN_BAUD 19200
#define CAN_BAUD 500000
#define CAN_TX_INTERVAL_MS 20  // 50Hz CAN update rate

// LIN Frame IDs from LDF (Vehicle ECU will send these)
#define LIN_FRAME_ID_BLW_F_STAT 0x32  // 50 decimal - BLDC Status Response
#define LIN_FRAME_ID_BLW_F_RQ   0x33  // 51 decimal - BLDC Command (ECU sends)

// CAN Message IDs
#define CAN_MSG_BLDC_STATUS_1   0x100  // 256
#define CAN_MSG_BLDC_STATUS_2   0x101  // 257
#define CAN_MSG_GATEWAY_INFO    0x102  // 258

// Pin definitions
#define LIN_RX_PIN PA3  // Only RX needed for passive sniffing
#define CAN_RX_PIN PB8
#define CAN_TX_PIN PB9
#define LED_PIN LED_BUILTIN

// LIN timing (microseconds)
#define LIN_BREAK_MIN_US 676   // Minimum break duration (13 bits at 19200 baud)
#define LIN_BYTE_TIME_US 520   // Time for one byte at 19200 baud
#define LIN_FRAME_TIMEOUT_MS 50 // Maximum time to receive complete frame

// ============================================================================
// TYPE DEFINITIONS
// ============================================================================

// LIN frame structure
typedef struct {
  uint8_t sync;
  uint8_t pid;           // Protected ID
  uint8_t id;            // Frame ID (6 bits)
  uint8_t len;
  uint8_t data[8];
  uint8_t checksum;
  bool valid;
} LIN_Frame_t;

// CAN frame structure
typedef struct {
  uint32_t id;
  uint8_t len;
  uint8_t data[8];
} CAN_Frame_t;

// BLDC Status structure (parsed from LIN)
typedef struct {
  uint8_t current;              // BLW_F_Curr_ST3
  uint8_t rpm;                  // BLW_F_RPM_ST3
  uint8_t temperature;          // BLW_F_Int_Temp_ST3
  uint16_t supplyVoltage;       // BLW_F_SupVolt_ST3 (10 bits)
  uint8_t responseError;        // RsErr_BLW_F_ST3
  uint8_t internalStatus;       // BLW_F_Int_Stat_ST3 (3 bits)
  uint8_t currentLimitStatus;   // BLW_F_Curr_Lim_Stat_ST3
  uint8_t voltageStatus;        // BLW_F_Volt_Stat_ST3
  uint8_t shortCircuitStatus;   // BLW_F_ShortCirc_Stat_ST3
  uint8_t blockStatus;          // BLW_F_Block_Stat_ST3
  uint8_t tempDeratStatus;      // BLW_F_TempDerat_Stat_ST3
  uint8_t rotationDirection;    // BLW_F_RotDir_Stat_ST3
  uint8_t internalErrorStatus;  // BLW_F_IntErr_Stat_ST3
  uint8_t voltDeratStatus;      // BLW_F_VoltDerat_Stat_ST3
  uint8_t windmillStatus;       // BLW_F_WindMill_Stat_ST3
  uint8_t wakeupStatus;         // WakeupStat_BLW_F_ST3
  uint32_t lastUpdate;          // Timestamp of last update
  bool dataValid;               // True if we've received valid data
} BLDC_Status_t;

// Gateway statistics
typedef struct {
  uint16_t frameCounter;
  uint16_t errorCounter;
  uint16_t checksumErrors;
  uint16_t timeoutErrors;
  uint8_t status;
  uint32_t lastFrameTime;
} Gateway_Stats_t;

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
HardwareSerial LinSerial(LIN_RX_PIN, PA2);  // PA2 unused but required for constructor
BLDC_Status_t bldcStatus = {0};
Gateway_Stats_t gatewayStats = {0};
uint32_t lastCANTxTime = 0;
volatile bool newDataAvailable = false;

// CAN HAL handles
CAN_HandleTypeDef hcan1;

// ============================================================================
// LIN SNIFFER FUNCTIONS
// ============================================================================

/**
 * @brief Calculate LIN 2.x Enhanced Checksum
 */
uint8_t LIN_CalculateChecksum(uint8_t pid, uint8_t* data, uint8_t len) {
  uint16_t sum = pid;  // Include protected ID in checksum
  for (uint8_t i = 0; i < len; i++) {
    sum += data[i];
    if (sum > 0xFF) {
      sum -= 0xFF;  // Carry handling
    }
  }
  return (uint8_t)(~sum);
}

/**
 * @brief Extract Frame ID from Protected ID
 */
uint8_t LIN_GetFrameID(uint8_t pid) {
  return pid & 0x3F;  // Lower 6 bits
}

/**
 * @brief Verify Protected ID parity bits
 */
bool LIN_VerifyPID(uint8_t pid) {
  uint8_t id = pid & 0x3F;
  uint8_t p0 = (pid >> 6) & 0x01;
  uint8_t p1 = (pid >> 7) & 0x01;
  
  uint8_t p0_calc = (id ^ (id >> 1) ^ (id >> 2) ^ (id >> 4)) & 0x01;
  uint8_t p1_calc = ~((id >> 1) ^ (id >> 3) ^ (id >> 4) ^ (id >> 5)) & 0x01;
  
  return (p0 == p0_calc) && (p1 == p1_calc);
}

/**
 * @brief Detect LIN break field (low voltage for extended period)
 */
bool LIN_DetectBreak() {
  // Look for 0x00 byte which indicates break/sync at different baud rate
  // This is a simplified detection - proper break is 13+ bit times low
  
  if (LinSerial.available()) {
    uint8_t byte = LinSerial.peek();
    if (byte == 0x00) {
      LinSerial.read(); // Consume break byte
      return true;
    }
  }
  return false;
}

/**
 * @brief Wait for specific byte with timeout
 */
bool LIN_WaitForByte(uint8_t* byte, uint32_t timeout_us) {
  uint32_t start = micros();
  while (micros() - start < timeout_us) {
    if (LinSerial.available()) {
      *byte = LinSerial.read();
      return true;
    }
  }
  return false;
}

/**
 * @brief Passive LIN frame sniffer
 * Listens to LIN bus and captures frames sent by vehicle ECU
 */
bool LIN_SniffFrame(LIN_Frame_t* frame) {
  static uint8_t state = 0;  // 0=waiting for break, 1=reading sync, 2=reading PID, 3=reading data
  static uint32_t frameStartTime = 0;
  uint8_t byte;
  
  // Check for frame timeout
  if (state > 0 && (millis() - frameStartTime) > LIN_FRAME_TIMEOUT_MS) {
    state = 0;
    gatewayStats.timeoutErrors++;
    #ifdef DEBUG_SERIAL
    Serial.println("LIN Frame Timeout");
    #endif
    return false;
  }
  
  switch (state) {
    case 0:  // Waiting for sync byte (0x55)
      if (LinSerial.available()) {
        byte = LinSerial.read();
        if (byte == 0x55) {
          frame->sync = byte;
          state = 1;
          frameStartTime = millis();
        }
      }
      break;
      
    case 1:  // Reading Protected ID
      if (LIN_WaitForByte(&byte, 2000)) {  // 2ms timeout
        frame->pid = byte;
        frame->id = LIN_GetFrameID(byte);
        
        if (!LIN_VerifyPID(byte)) {
          #ifdef DEBUG_SERIAL
          Serial.print("Invalid PID: 0x");
          Serial.println(byte, HEX);
          #endif
          state = 0;
          gatewayStats.errorCounter++;
          return false;
        }
        
        // Determine frame length based on ID
        if (frame->id == LIN_FRAME_ID_BLW_F_STAT) {
          frame->len = 8;
          state = 2;
        } else {
          // Not interested in this frame, wait for next
          state = 0;
        }
      }
      break;
      
    case 2:  // Reading data bytes
      {
        static uint8_t dataIndex = 0;
        if (state == 2 && dataIndex == 0) {
          dataIndex = 0;  // Reset on entry to state
        }
        
        while (dataIndex < frame->len) {
          if (LIN_WaitForByte(&byte, 2000)) {
            frame->data[dataIndex++] = byte;
          } else {
            state = 0;
            dataIndex = 0;
            gatewayStats.timeoutErrors++;
            return false;
          }
        }
        
        // All data received, now get checksum
        dataIndex = 0;
        state = 3;
      }
      break;
      
    case 3:  // Reading checksum
      if (LIN_WaitForByte(&byte, 2000)) {
        frame->checksum = byte;
        
        // Verify checksum
        uint8_t calculated = LIN_CalculateChecksum(frame->pid, frame->data, frame->len);
        if (calculated == frame->checksum) {
          frame->valid = true;
          state = 0;
          return true;  // Complete valid frame received!
        } else {
          #ifdef DEBUG_SERIAL
          Serial.print("Checksum Error - Expected: 0x");
          Serial.print(calculated, HEX);
          Serial.print(", Got: 0x");
          Serial.println(byte, HEX);
          #endif
          gatewayStats.checksumErrors++;
          state = 0;
          return false;
        }
      }
      state = 0;
      break;
  }
  
  return false;
}

/**
 * @brief Parse BLDC Status from LIN frame data
 */
void ParseBLDCStatus(uint8_t* data) {
  // Byte 0: BLW_F_Curr_ST3 (bits 0-7)
  bldcStatus.current = data[0];
  
  // Byte 1: BLW_F_RPM_ST3 (bits 8-15)
  bldcStatus.rpm = data[1];
  
  // Byte 2: BLW_F_Int_Temp_ST3 (bits 16-23)
  bldcStatus.temperature = data[2];
  
  // Byte 3-4: BLW_F_SupVolt_ST3 (bits 24-33, 10 bits)
  bldcStatus.supplyVoltage = (data[3] | ((data[4] & 0x03) << 8));
  
  // Byte 4: RsErr_BLW_F_ST3 (bit 34)
  bldcStatus.responseError = (data[4] >> 2) & 0x01;
  
  // Byte 4: BLW_F_Int_Stat_ST3 (bits 35-37, 3 bits)
  bldcStatus.internalStatus = (data[4] >> 3) & 0x07;
  
  // Byte 4-5: BLW_F_Curr_Lim_Stat_ST3 (bits 38-39)
  bldcStatus.currentLimitStatus = (data[4] >> 6) | ((data[5] & 0x01) << 1);
  
  // Byte 5: BLW_F_Volt_Stat_ST3 (bits 40-41)
  bldcStatus.voltageStatus = (data[5] >> 1) & 0x03;
  
  // Byte 5: BLW_F_ShortCirc_Stat_ST3 (bits 42-43)
  bldcStatus.shortCircuitStatus = (data[5] >> 3) & 0x03;
  
  // Byte 5: BLW_F_Block_Stat_ST3 (bits 44-45)
  bldcStatus.blockStatus = (data[5] >> 5) & 0x03;
  
  // Byte 5-6: BLW_F_TempDerat_Stat_ST3 (bits 46-47)
  bldcStatus.tempDeratStatus = (data[5] >> 7) | ((data[6] & 0x01) << 1);
  
  // Byte 6: BLW_F_RotDir_Stat_ST3 (bits 48-49)
  bldcStatus.rotationDirection = (data[6] >> 1) & 0x03;
  
  // Byte 6: BLW_F_IntErr_Stat_ST3 (bits 50-51)
  bldcStatus.internalErrorStatus = (data[6] >> 3) & 0x03;
  
  // Byte 6: BLW_F_VoltDerat_Stat_ST3 (bits 52-53)
  bldcStatus.voltDeratStatus = (data[6] >> 5) & 0x03;
  
  // Byte 6-7: BLW_F_WindMill_Stat_ST3 (bits 54-55)
  bldcStatus.windmillStatus = (data[6] >> 7) | ((data[7] & 0x01) << 1);
  
  // Byte 7: WakeupStat_BLW_F_ST3 (bits 56-57)
  bldcStatus.wakeupStatus = (data[7] >> 1) & 0x03;
  
  // Update metadata
  bldcStatus.lastUpdate = millis();
  bldcStatus.dataValid = true;
  newDataAvailable = true;
}

// ============================================================================
// CAN FUNCTIONS
// ============================================================================

/**
 * @brief Initialize CAN peripheral
 */
void CAN_Init() {
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 9;  // For 500kbps with 90MHz APB1
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  
  if (HAL_CAN_Init(&hcan1) != HAL_OK) {
    #ifdef DEBUG_SERIAL
    Serial.println("CAN Init Failed!");
    #endif
    gatewayStats.status = 5;  // CAN_ERROR
    return;
  }
  
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    #ifdef DEBUG_SERIAL
    Serial.println("CAN Start Failed!");
    #endif
    gatewayStats.status = 5;
    return;
  }
  
  #ifdef DEBUG_SERIAL
  Serial.println("CAN Initialized - Ready to transmit");
  #endif
  gatewayStats.status = 2;  // CAN_READY
}

/**
 * @brief Send CAN frame
 */
bool CAN_SendFrame(CAN_Frame_t* frame) {
  CAN_TxHeaderTypeDef txHeader;
  uint32_t txMailbox;
  
  txHeader.StdId = frame->id;
  txHeader.ExtId = 0;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = frame->len;
  txHeader.TransmitGlobalTime = DISABLE;
  
  if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, frame->data, &txMailbox) != HAL_OK) {
    gatewayStats.errorCounter++;
    return false;
  }
  
  return true;
}

/**
 * @brief Build and send BLDC Status 1 CAN message
 */
void SendCANMessage_BLDCStatus1() {
  CAN_Frame_t frame;
  frame.id = CAN_MSG_BLDC_STATUS_1;
  frame.len = 8;
  
  // Byte 0: BLDC_Current (8 bits)
  frame.data[0] = bldcStatus.current;
  
  // Byte 1: BLDC_RPM (8 bits)
  frame.data[1] = bldcStatus.rpm;
  
  // Byte 2: BLDC_Temperature (8 bits)
  frame.data[2] = bldcStatus.temperature;
  
  // Byte 3-4: BLDC_SupplyVoltage (10 bits)
  frame.data[3] = bldcStatus.supplyVoltage & 0xFF;
  frame.data[4] = (bldcStatus.supplyVoltage >> 8) & 0x03;
  
  // Byte 4: BLDC_ResponseError (1 bit at bit 34)
  frame.data[4] |= (bldcStatus.responseError & 0x01) << 2;
  
  // Byte 4: BLDC_InternalStatus (3 bits at bits 35-37)
  frame.data[4] |= (bldcStatus.internalStatus & 0x07) << 3;
  
  // Byte 4-5: BLDC_CurrentLimitStatus (2 bits at bits 38-39)
  frame.data[4] |= (bldcStatus.currentLimitStatus & 0x03) << 6;
  frame.data[5] = (bldcStatus.currentLimitStatus >> 2) & 0x01;
  
  // Byte 5: BLDC_VoltageStatus (2 bits at bits 40-41)
  frame.data[5] |= (bldcStatus.voltageStatus & 0x03) << 1;
  
  // Fill remaining bytes
  frame.data[6] = 0;
  frame.data[7] = 0;
  
  CAN_SendFrame(&frame);
}

/**
 * @brief Build and send BLDC Status 2 CAN message
 */
void SendCANMessage_BLDCStatus2() {
  CAN_Frame_t frame;
  frame.id = CAN_MSG_BLDC_STATUS_2;
  frame.len = 8;
  
  // Pack all status bits
  frame.data[0] = bldcStatus.shortCircuitStatus & 0x03;
  frame.data[0] |= (bldcStatus.blockStatus & 0x03) << 2;
  frame.data[0] |= (bldcStatus.tempDeratStatus & 0x03) << 4;
  frame.data[0] |= (bldcStatus.rotationDirection & 0x03) << 6;
  
  frame.data[1] = bldcStatus.internalErrorStatus & 0x03;
  frame.data[1] |= (bldcStatus.voltDeratStatus & 0x03) << 2;
  frame.data[1] |= (bldcStatus.windmillStatus & 0x03) << 4;
  frame.data[1] |= (bldcStatus.wakeupStatus & 0x03) << 6;
  
  frame.data[2] = 0;
  frame.data[3] = 0;
  frame.data[4] = 0;
  frame.data[5] = 0;
  frame.data[6] = 0;
  frame.data[7] = 0;
  
  CAN_SendFrame(&frame);
}

/**
 * @brief Build and send Gateway Info CAN message
 */
void SendCANMessage_GatewayInfo() {
  CAN_Frame_t frame;
  frame.id = CAN_MSG_GATEWAY_INFO;
  frame.len = 8;
  
  // Frame counter (16 bits)
  frame.data[0] = gatewayStats.frameCounter & 0xFF;
  frame.data[1] = (gatewayStats.frameCounter >> 8) & 0xFF;
  
  // Error counter (16 bits) - Total errors
  uint16_t totalErrors = gatewayStats.errorCounter + gatewayStats.checksumErrors + gatewayStats.timeoutErrors;
  frame.data[2] = totalErrors & 0xFF;
  frame.data[3] = (totalErrors >> 8) & 0xFF;
  
  // Gateway status
  frame.data[4] = gatewayStats.status;
  
  // Time since last LIN frame (seconds, capped at 255)
  uint8_t timeSinceFrame = min(255, (millis() - bldcStatus.lastUpdate) / 1000);
  frame.data[5] = bldcStatus.dataValid ? timeSinceFrame : 255;
  
  // Checksum errors
  frame.data[6] = min(255, (int)gatewayStats.checksumErrors);
  
  // Timeout errors
  frame.data[7] = min(255, (int)gatewayStats.timeoutErrors);
  
  CAN_SendFrame(&frame);
}

// ============================================================================
// MAIN FUNCTIONS
// ============================================================================

void setup() {
  // Initialize debug serial
  #ifdef DEBUG_SERIAL
  Serial.begin(115200);
  delay(1000);
  Serial.println("================================================");
  Serial.println("   CAN-LIN PASSIVE GATEWAY (SNIFFER MODE)");
  Serial.println("================================================");
  Serial.println("BLDC Motor Monitoring for Acoustic Testing");
  Serial.println("Mode: PASSIVE - Monitoring LIN bus only");
  Serial.println("ECU controls motor, Gateway listens & forwards");
  Serial.println("================================================");
  #endif
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize LIN (RX only for passive sniffing)
  LinSerial.begin(LIN_BAUD);
  gatewayStats.status = 1;  // LIN_READY
  
  #ifdef DEBUG_SERIAL
  Serial.println("[LIN] Initialized in PASSIVE mode at 19200 baud");
  Serial.println("[LIN] Listening for frames from Vehicle ECU...");
  #endif
  
  // Initialize CAN
  CAN_Init();
  
  #ifdef DEBUG_SERIAL
  Serial.println("[INFO] Gateway Ready - Waiting for LIN traffic");
  Serial.println("================================================\n");
  #endif
  
  gatewayStats.status = 3;  // RUNNING
  lastCANTxTime = millis();
  bldcStatus.dataValid = false;
}

void loop() {
  static uint32_t lastStatusPrint = 0;
  uint32_t currentTime = millis();
  
  // Continuously sniff LIN bus for frames
  LIN_Frame_t frame;
  if (LIN_SniffFrame(&frame)) {
    // Valid frame received from LIN bus
    if (frame.id == LIN_FRAME_ID_BLW_F_STAT) {
      // This is the BLDC status frame we're interested in
      ParseBLDCStatus(frame.data);
      gatewayStats.frameCounter++;
      gatewayStats.lastFrameTime = currentTime;
      
      // Toggle LED to show activity
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      
      #ifdef DEBUG_SERIAL
      if (gatewayStats.frameCounter % 25 == 1) {  // Print every 0.5 second (approx)
        float rpmReal = bldcStatus.rpm * 25.0;
        int8_t tempReal = (int8_t)bldcStatus.temperature - 40;
        float currentReal = bldcStatus.current * 0.15;
        float voltageReal = bldcStatus.supplyVoltage * 0.1;
        
        Serial.print("[DATA] RPM: ");
        Serial.print(rpmReal, 0);
        Serial.print(" | Temp: ");
        Serial.print(tempReal);
        Serial.print("°C | Current: ");
        Serial.print(currentReal, 2);
        Serial.print("A | Voltage: ");
        Serial.print(voltageReal, 1);
        Serial.print("V | Status: ");
        Serial.println(bldcStatus.internalStatus);
      }
      #endif
    }
  }
  
  // Send CAN messages at fixed rate (20ms = 50Hz)
  if (currentTime - lastCANTxTime >= CAN_TX_INTERVAL_MS) {
    lastCANTxTime = currentTime;
    
    if (bldcStatus.dataValid) {
      // Send BLDC data to CAN bus
      SendCANMessage_BLDCStatus1();
      SendCANMessage_BLDCStatus2();
    }
    
    // Send gateway info every 100ms (every 5th cycle)
    static uint8_t infoCounter = 0;
    if (++infoCounter >= 5) {
      infoCounter = 0;
      SendCANMessage_GatewayInfo();
    }
  }
  
  // Print statistics every 5 seconds
  #ifdef DEBUG_SERIAL
  if (currentTime - lastStatusPrint >= 5000) {
    lastStatusPrint = currentTime;
    
    Serial.println("\n--- Gateway Statistics ---");
    Serial.print("LIN Frames: ");
    Serial.print(gatewayStats.frameCounter);
    Serial.print(" | Checksum Errors: ");
    Serial.print(gatewayStats.checksumErrors);
    Serial.print(" | Timeouts: ");
    Serial.println(gatewayStats.timeoutErrors);
    
    if (bldcStatus.dataValid) {
      uint32_t age = (currentTime - bldcStatus.lastUpdate) / 1000;
      Serial.print("Last data: ");
      Serial.print(age);
      Serial.println(" seconds ago");
    } else {
      Serial.println("Status: Waiting for first LIN frame...");
    }
    Serial.println("--------------------------\n");
  }
  #endif
  
  // Check for stale data (no LIN frames for >2 seconds)
  if (bldcStatus.dataValid && (currentTime - bldcStatus.lastUpdate) > 2000) {
    digitalWrite(LED_PIN, LOW);  // Solid off = no data
    
    #ifdef DEBUG_SERIAL
    static bool warningPrinted = false;
    if (!warningPrinted) {
      Serial.println("[WARNING] No LIN data received for >2 seconds");
      Serial.println("[WARNING] Check: Vehicle ECU powered? LIN wiring? BLDC active?");
      warningPrinted = true;
    }
    #endif
  }
}

// ============================================================================
// HAL MSP Functions (required by STM32 HAL)
// ============================================================================

extern "C" {

void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  if (hcan->Instance == CAN1) {
    // Enable clocks
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    // Configure CAN pins
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan) {
  if (hcan->Instance == CAN1) {
    __HAL_RCC_CAN1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);
  }
}

}  // extern "C"