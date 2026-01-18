#!/usr/bin/env python3
"""
CAN-LIN Gateway Configuration Tool
Professional GUI for generating STM32 gateway firmware from LDF/DBC files

Author: Gateway Config Tool
Version: 1.0.0
Date: 2026-01-17
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext
import re
import os
from datetime import datetime
from pathlib import Path

class LDFParser:
    """Parse LIN Description Files (.ldf)"""
    
    def __init__(self, filepath):
        self.filepath = filepath
        self.signals = {}
        self.frames = {}
        self.nodes = {}
        self.protocol_version = "2.0"
        self.speed = 19200
        
    def parse(self):
        """Parse LDF file and extract signals, frames, nodes"""
        try:
            with open(self.filepath, 'r', encoding='utf-8', errors='ignore') as f:
                content = f.read()
            
            # Parse speed
            speed_match = re.search(r'LIN_speed\s*=\s*(\d+\.?\d*)\s*kbps', content)
            if speed_match:
                self.speed = int(float(speed_match.group(1)) * 1000)
            
            # Parse nodes
            nodes_match = re.search(r'Slaves:\s*([^;]+);', content)
            if nodes_match:
                nodes_text = nodes_match.group(1)
                node_list = re.findall(r'(\w+)', nodes_text)
                for node in node_list:
                    if node not in ['Slaves']:
                        self.nodes[node] = {'nad': 0x00, 'frames': []}
            
            # Parse signals
            signals_section = re.search(r'Signals\s*\{([^}]+)\}', content, re.DOTALL)
            if signals_section:
                signal_lines = signals_section.group(1).strip().split('\n')
                for line in signal_lines:
                    sig_match = re.match(r'\s*(\w+):\s*(\d+),\s*(\d+),\s*(\w+),\s*(\w+)', line)
                    if sig_match:
                        name, size, init_val, publisher, subscriber = sig_match.groups()
                        self.signals[name] = {
                            'size': int(size),
                            'init_value': int(init_val),
                            'publisher': publisher,
                            'subscriber': subscriber
                        }
            
            # Parse frames
            frames_section = re.search(r'Frames\s*\{([^}]+)\}', content, re.DOTALL)
            if frames_section:
                frame_text = frames_section.group(1)
                # Match frame definitions
                frame_pattern = r'(\w+):\s*(\d+),\s*(\w+),\s*(\d+)\s*\{([^}]+)\}'
                for match in re.finditer(frame_pattern, frame_text, re.DOTALL):
                    frame_name, frame_id, publisher, length, signals_text = match.groups()
                    
                    frame_signals = []
                    sig_pattern = r'(\w+),\s*(\d+)'
                    for sig_match in re.finditer(sig_pattern, signals_text):
                        sig_name, bit_offset = sig_match.groups()
                        frame_signals.append({
                            'name': sig_name,
                            'offset': int(bit_offset)
                        })
                    
                    self.frames[frame_name] = {
                        'id': int(frame_id),
                        'publisher': publisher,
                        'length': int(length),
                        'signals': frame_signals
                    }
            
            return True
        except Exception as e:
            print(f"LDF Parse Error: {e}")
            return False

class DBCParser:
    """Parse CAN Database files (.dbc)"""
    
    def __init__(self, filepath):
        self.filepath = filepath
        self.messages = {}
        self.signals = {}
        self.nodes = []
        self.baudrate = 500000
        
    def parse(self):
        """Parse DBC file and extract messages and signals"""
        try:
            with open(self.filepath, 'r', encoding='utf-8', errors='ignore') as f:
                content = f.read()
            
            # Parse nodes
            nodes_match = re.search(r'BU_:\s*([^\n]+)', content)
            if nodes_match:
                self.nodes = nodes_match.group(1).strip().split()
            
            # Parse messages
            msg_pattern = r'BO_\s+(\d+)\s+(\w+):\s*(\d+)\s+(\w+)'
            for msg_match in re.finditer(msg_pattern, content):
                msg_id, msg_name, msg_len, node = msg_match.groups()
                self.messages[msg_name] = {
                    'id': int(msg_id),
                    'length': int(msg_len),
                    'sender': node,
                    'signals': []
                }
            
            # Parse signals
            sig_pattern = r'SG_\s+(\w+)\s*:\s*(\d+)\|(\d+)@([01])([+-])\s*\(([^,]+),([^)]+)\)\s*\[([^|]+)\|([^\]]+)\]\s*"([^"]*)"'
            current_message = None
            
            for line in content.split('\n'):
                # Track current message context
                msg_match = re.match(r'BO_\s+\d+\s+(\w+):', line)
                if msg_match:
                    current_message = msg_match.group(1)
                
                # Parse signal
                sig_match = re.match(sig_pattern, line.strip())
                if sig_match and current_message:
                    sig_name, start_bit, length, byte_order, value_type, factor, offset, min_val, max_val, unit = sig_match.groups()
                    
                    signal_info = {
                        'name': sig_name,
                        'start_bit': int(start_bit),
                        'length': int(length),
                        'byte_order': 'little' if byte_order == '1' else 'big',
                        'signed': value_type == '-',
                        'factor': float(factor),
                        'offset': float(offset),
                        'min': float(min_val),
                        'max': float(max_val),
                        'unit': unit
                    }
                    
                    self.signals[sig_name] = signal_info
                    if current_message in self.messages:
                        self.messages[current_message]['signals'].append(signal_info)
            
            return True
        except Exception as e:
            print(f"DBC Parse Error: {e}")
            return False

class CodeGenerator:
    """Generate STM32 gateway code"""
    
    @staticmethod
    def generate_lin_to_can(ldf_data, config):
        """Generate LIN to CAN sniffer code"""
        
        # Find first slave frame
        slave_frames = [f for fname, f in ldf_data.frames.items() 
                       if f['publisher'] != 'Klima' and f['publisher'] in ldf_data.nodes]
        
        if not slave_frames:
            return None
        
        primary_frame = slave_frames[0]
        frame_name = [k for k, v in ldf_data.frames.items() if v == primary_frame][0]
        
        code = f'''/**
 * @file main.cpp
 * @brief CAN-LIN Passive Gateway - LIN to CAN Mode
 * @generated {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
 * 
 * Mode: LIN Sniffer → CAN Transmitter
 * LIN Baud: {ldf_data.speed} bps
 * CAN Baud: {config['can_baudrate']} bps
 * Primary Frame: {frame_name} (ID: 0x{primary_frame['id']:02X})
 */

#include <Arduino.h>
#include <HardwareSerial.h>

// Configuration
#define LIN_BAUD {ldf_data.speed}
#define CAN_BAUD {config['can_baudrate']}
#define CAN_TX_INTERVAL_MS {config['can_interval']}
#define LIN_FRAME_ID 0x{primary_frame['id']:02X}

// Pin definitions
#define LIN_RX_PIN PA3
#define CAN_RX_PIN PB8
#define CAN_TX_PIN PB9
#define LED_PIN LED_BUILTIN

// CAN message IDs
#define CAN_MSG_STATUS_1   0x100
#define CAN_MSG_STATUS_2   0x101
#define CAN_MSG_GATEWAY    0x102

// Global variables
HardwareSerial LinSerial(LIN_RX_PIN, PA2);
CAN_HandleTypeDef hcan1;

typedef struct {{
{CodeGenerator._generate_signal_struct(primary_frame['signals'], ldf_data.signals)}
    uint32_t lastUpdate;
    bool dataValid;
}} DeviceStatus_t;

DeviceStatus_t deviceStatus = {{0}};

// LIN Functions
uint8_t LIN_CalculateChecksum(uint8_t pid, uint8_t* data, uint8_t len) {{
    uint16_t sum = pid;
    for (uint8_t i = 0; i < len; i++) {{
        sum += data[i];
        if (sum > 0xFF) sum -= 0xFF;
    }}
    return (uint8_t)(~sum);
}}

bool LIN_SniffFrame(uint8_t* data, uint8_t len) {{
    static uint8_t state = 0;
    static uint8_t frameData[8];
    static uint8_t dataIdx = 0;
    
    if (!LinSerial.available()) return false;
    
    uint8_t byte = LinSerial.read();
    
    switch(state) {{
        case 0: // Wait for sync
            if (byte == 0x55) state = 1;
            break;
        case 1: // Read PID
            if ((byte & 0x3F) == LIN_FRAME_ID) {{
                state = 2;
                dataIdx = 0;
            }} else {{
                state = 0;
            }}
            break;
        case 2: // Read data
            frameData[dataIdx++] = byte;
            if (dataIdx >= len) state = 3;
            break;
        case 3: // Read checksum
            state = 0;
            // Verify and copy data
            memcpy(data, frameData, len);
            return true;
    }}
    
    return false;
}}

void ParseDeviceStatus(uint8_t* data) {{
{CodeGenerator._generate_parse_code(primary_frame['signals'], ldf_data.signals)}
    deviceStatus.lastUpdate = millis();
    deviceStatus.dataValid = true;
}}

// CAN Functions
void CAN_Init() {{
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = {config['can_prescaler']};
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
    
    HAL_CAN_Init(&hcan1);
    HAL_CAN_Start(&hcan1);
}}

void CAN_SendStatus() {{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t canData[8] = {{0}};
    
    txHeader.StdId = CAN_MSG_STATUS_1;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 8;
    
{CodeGenerator._generate_can_pack_code(primary_frame['signals'][:4] if len(primary_frame['signals']) > 4 else primary_frame['signals'], ldf_data.signals)}
    
    HAL_CAN_AddTxMessage(&hcan1, &txHeader, canData, &txMailbox);
}}

void setup() {{
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    
    LinSerial.begin(LIN_BAUD);
    CAN_Init();
    
    Serial.println("Gateway Ready - LIN to CAN Mode");
    Serial.println("Monitoring LIN Frame ID: 0x{primary_frame['id']:02X}");
}}

void loop() {{
    static uint32_t lastCANTx = 0;
    uint8_t linData[{primary_frame['length']}];
    
    // Sniff LIN bus
    if (LIN_SniffFrame(linData, {primary_frame['length']})) {{
        ParseDeviceStatus(linData);
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }}
    
    // Send CAN at fixed rate
    if (millis() - lastCANTx >= CAN_TX_INTERVAL_MS) {{
        lastCANTx = millis();
        if (deviceStatus.dataValid) {{
            CAN_SendStatus();
        }}
    }}
}}

// HAL MSP Functions
extern "C" {{
void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan) {{
    if (hcan->Instance == CAN1) {{
        __HAL_RCC_CAN1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        
        GPIO_InitTypeDef GPIO_InitStruct = {{0}};
        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }}
}}
}}
'''
        return code
    
    @staticmethod
    def generate_can_to_lin(dbc_data, config):
        """Generate CAN to LIN forwarder code"""
        
        if not dbc_data.messages:
            return None
        
        # Get first message
        msg_name = list(dbc_data.messages.keys())[0]
        msg = dbc_data.messages[msg_name]
        
        code = f'''/**
 * @file main.cpp
 * @brief CAN-LIN Gateway - CAN to LIN Mode
 * @generated {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
 * 
 * Mode: CAN Receiver → LIN Transmitter
 * CAN Baud: {config['can_baudrate']} bps
 * LIN Baud: {config['lin_baudrate']} bps
 * Primary Message: {msg_name} (ID: 0x{msg['id']:03X})
 */

#include <Arduino.h>
#include <HardwareSerial.h>

// Configuration
#define CAN_BAUD {config['can_baudrate']}
#define LIN_BAUD {config['lin_baudrate']}
#define CAN_MSG_ID 0x{msg['id']:03X}
#define LIN_FRAME_ID 0x{config.get('lin_frame_id', 0x10):02X}

// Pin definitions
#define LIN_TX_PIN PA2
#define LIN_RX_PIN PA3
#define CAN_RX_PIN PB8
#define CAN_TX_PIN PB9

// Global variables
HardwareSerial LinSerial(LIN_RX_PIN, LIN_TX_PIN);
CAN_HandleTypeDef hcan1;

typedef struct {{
{CodeGenerator._generate_signal_struct_from_dbc(msg['signals'][:4] if len(msg['signals']) > 4 else msg['signals'])}
}} CANMessage_t;

CANMessage_t canMessage = {{0}};
bool newDataReceived = false;

// LIN Functions
void LIN_SendBreak() {{
    LinSerial.end();
    LinSerial.begin(LIN_BAUD / 2);
    LinSerial.write(0x00);
    LinSerial.flush();
    delayMicroseconds(750);
    LinSerial.end();
    LinSerial.begin(LIN_BAUD);
}}

uint8_t LIN_CalculateProtectedID(uint8_t id) {{
    uint8_t p0 = (id ^ (id >> 1) ^ (id >> 2) ^ (id >> 4)) & 0x01;
    uint8_t p1 = ~((id >> 1) ^ (id >> 3) ^ (id >> 4) ^ (id >> 5)) & 0x01;
    return id | (p0 << 6) | (p1 << 7);
}}

uint8_t LIN_CalculateChecksum(uint8_t pid, uint8_t* data, uint8_t len) {{
    uint16_t sum = pid;
    for (uint8_t i = 0; i < len; i++) {{
        sum += data[i];
        if (sum > 0xFF) sum -= 0xFF;
    }}
    return (uint8_t)(~sum);
}}

void LIN_SendFrame(uint8_t frameId, uint8_t* data, uint8_t len) {{
    uint8_t pid = LIN_CalculateProtectedID(frameId);
    uint8_t checksum = LIN_CalculateChecksum(pid, data, len);
    
    LIN_SendBreak();
    LinSerial.write(0x55);  // Sync
    LinSerial.write(pid);
    LinSerial.write(data, len);
    LinSerial.write(checksum);
    LinSerial.flush();
}}

// CAN Functions
void CAN_Init() {{
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = {config['can_prescaler']};
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
    
    HAL_CAN_Init(&hcan1);
    
    // Configure CAN filter
    CAN_FilterTypeDef filter;
    filter.FilterIdHigh = CAN_MSG_ID << 5;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = 0xFFE0;
    filter.FilterMaskIdLow = 0;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterActivation = ENABLE;
    HAL_CAN_ConfigFilter(&hcan1, &filter);
    
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}}

void ParseCANMessage(uint8_t* data) {{
{CodeGenerator._generate_can_parse_code(msg['signals'][:4] if len(msg['signals']) > 4 else msg['signals'])}
}}

void setup() {{
    Serial.begin(115200);
    LinSerial.begin(LIN_BAUD);
    CAN_Init();
    
    Serial.println("Gateway Ready - CAN to LIN Mode");
    Serial.println("Listening for CAN ID: 0x{msg['id']:03X}");
}}

void loop() {{
    if (newDataReceived) {{
        newDataReceived = false;
        
        // Pack data to LIN format
        uint8_t linData[8] = {{0}};
{CodeGenerator._generate_lin_pack_code(msg['signals'][:4] if len(msg['signals']) > 4 else msg['signals'])}
        
        // Send to LIN bus
        LIN_SendFrame(LIN_FRAME_ID, linData, 8);
        
        Serial.println("CAN → LIN forwarded");
    }}
    delay(1);
}}

// CAN RX Interrupt
extern "C" {{
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {{
        if (rxHeader.StdId == CAN_MSG_ID) {{
            ParseCANMessage(rxData);
            newDataReceived = true;
        }}
    }}
}}

void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan) {{
    if (hcan->Instance == CAN1) {{
        __HAL_RCC_CAN1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        
        GPIO_InitTypeDef GPIO_InitStruct = {{0}};
        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        
        HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    }}
}}
}}
'''
        return code
    
    @staticmethod
    def _generate_signal_struct(signals, signal_defs):
        """Generate C struct fields from LIN signals"""
        code = ""
        for sig in signals:
            sig_name = sig['name']
            if sig_name in signal_defs:
                sig_def = signal_defs[sig_name]
                if sig_def['size'] <= 8:
                    code += f"    uint8_t {sig_name};\n"
                elif sig_def['size'] <= 16:
                    code += f"    uint16_t {sig_name};\n"
                else:
                    code += f"    uint32_t {sig_name};\n"
        return code
    
    @staticmethod
    def _generate_signal_struct_from_dbc(signals):
        """Generate C struct fields from DBC signals"""
        code = ""
        for sig in signals:
            if sig['length'] <= 8:
                code += f"    uint8_t {sig['name']};\n"
            elif sig['length'] <= 16:
                code += f"    uint16_t {sig['name']};\n"
            else:
                code += f"    uint32_t {sig['name']};\n"
        return code
    
    @staticmethod
    def _generate_parse_code(signals, signal_defs):
        """Generate parsing code for LIN frame"""
        code = ""
        for sig in signals:
            sig_name = sig['name']
            offset = sig['offset']
            byte_idx = offset // 8
            bit_idx = offset % 8
            
            if sig_name in signal_defs:
                sig_def = signal_defs[sig_name]
                size = sig_def['size']
                
                if size <= 8 and bit_idx == 0:
                    code += f"    deviceStatus.{sig_name} = data[{byte_idx}];\n"
                elif size <= 8:
                    code += f"    deviceStatus.{sig_name} = (data[{byte_idx}] >> {bit_idx}) & 0x{(1 << size) - 1:02X};\n"
                else:
                    # Multi-byte signal
                    code += f"    deviceStatus.{sig_name} = data[{byte_idx}];\n"
        
        return code
    
    @staticmethod
    def _generate_can_pack_code(signals, signal_defs):
        """Generate CAN packing code"""
        code = ""
        byte_idx = 0
        for sig in signals:
            sig_name = sig['name']
            if sig_name in signal_defs:
                code += f"    canData[{byte_idx}] = deviceStatus.{sig_name};\n"
                byte_idx += 1
        return code
    
    @staticmethod
    def _generate_can_parse_code(signals):
        """Generate CAN parsing code from DBC"""
        code = ""
        for i, sig in enumerate(signals):
            byte_idx = sig['start_bit'] // 8
            code += f"    canMessage.{sig['name']} = data[{byte_idx}];\n"
        return code
    
    @staticmethod
    def _generate_lin_pack_code(signals):
        """Generate LIN packing code from CAN data"""
        code = ""
        for i, sig in enumerate(signals):
            code += f"        linData[{i}] = canMessage.{sig['name']};\n"
        return code

class GatewayConfigGUI:
    """Main GUI Application"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("CAN-LIN Gateway Configuration Tool v1.0")
        self.root.geometry("1000x700")
        
        # Variables
        self.mode = tk.StringVar(value="lin_to_can")
        self.ldf_path = tk.StringVar()
        self.dbc_path = tk.StringVar()
        self.can_baudrate = tk.IntVar(value=500000)
        self.lin_baudrate = tk.IntVar(value=19200)
        self.can_interval = tk.IntVar(value=20)
        
        self.setup_ui()
        
    def setup_ui(self):
        """Setup GUI layout"""
        
        # Main container
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Title
        title = ttk.Label(main_frame, text="CAN-LIN Gateway Code Generator", 
                         font=('Arial', 16, 'bold'))
        title.grid(row=0, column=0, columnspan=3, pady=10)
        
        # Mode selection
        mode_frame = ttk.LabelFrame(main_frame, text="Gateway Mode", padding="10")
        mode_frame.grid(row=1, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Radiobutton(mode_frame, text="LIN Sniffer → CAN Transmitter (Passive Monitoring)", 
                       variable=self.mode, value="lin_to_can",
                       command=self.on_mode_change).grid(row=0, column=0, sticky=tk.W, padx=5)
        
        ttk.Radiobutton(mode_frame, text="CAN Receiver → LIN Transmitter (Active Control)", 
                       variable=self.mode, value="can_to_lin",
                       command=self.on_mode_change).grid(row=1, column=0, sticky=tk.W, padx=5)
        
        # File selection
        file_frame = ttk.LabelFrame(main_frame, text="Input Files", padding="10")
        file_frame.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        
        # LDF file
        self.ldf_label = ttk.Label(file_frame, text="LDF File (LIN):")
        self.ldf_label.grid(row=0, column=0, sticky=tk.W, pady=5)
        
        self.ldf_entry = ttk.Entry(file_frame, textvariable=self.ldf_path, width=50)
        self.ldf_entry.grid(row=0, column=1, padx=5)
        
        self.ldf_button = ttk.Button(file_frame, text="Browse...", command=self.browse_ldf)
        self.ldf_button.grid(row=0, column=2, padx=5)
        
        # DBC file
        self.dbc_label = ttk.Label(file_frame, text="DBC File (CAN):")
        self.dbc_label.grid(row=1, column=0, sticky=tk.W, pady=5)
        
        self.dbc_entry = ttk.Entry(file_frame, textvariable=self.dbc_path, width=50, state='disabled')
        self.dbc_entry.grid(row=1, column=1, padx=5)
        
        self.dbc_button = ttk.Button(file_frame, text="Browse...", command=self.browse_dbc, state='disabled')
        self.dbc_button.grid(row=1, column=2, padx=5)
        
        # Configuration
        config_frame = ttk.LabelFrame(main_frame, text="Gateway Configuration", padding="10")
        config_frame.grid(row=3, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Label(config_frame, text="CAN Baud Rate:").grid(row=0, column=0, sticky=tk.W)
        can_baud_combo = ttk.Combobox(config_frame, textvariable=self.can_baudrate, 
                                      values=[125000, 250000, 500000, 1000000], width=15)
        can_baud_combo.grid(row=0, column=1, padx=5, pady=2)
        ttk.Label(config_frame, text="bps").grid(row=0, column=2, sticky=tk.W)
        
        ttk.Label(config_frame, text="LIN Baud Rate:").grid(row=1, column=0, sticky=tk.W)
        lin_baud_combo = ttk.Combobox(config_frame, textvariable=self.lin_baudrate,
                                      values=[9600, 19200], width=15)
        lin_baud_combo.grid(row=1, column=1, padx=5, pady=2)
        ttk.Label(config_frame, text="bps").grid(row=1, column=2, sticky=tk.W)
        
        ttk.Label(config_frame, text="CAN TX Interval:").grid(row=2, column=0, sticky=tk.W)
        ttk.Spinbox(config_frame, from_=10, to=1000, textvariable=self.can_interval, width=15).grid(row=2, column=1, padx=5, pady=2)
        ttk.Label(config_frame, text="ms").grid(row=2, column=2, sticky=tk.W)
        
        # Buttons
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=4, column=0, columnspan=3, pady=10)
        
        ttk.Button(button_frame, text="Generate Code", command=self.generate_code, 
                  style='Accent.TButton').pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Save main.cpp", command=self.save_code).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Clear", command=self.clear_all).pack(side=tk.LEFT, padx=5)
        
        # Code preview
        preview_frame = ttk.LabelFrame(main_frame, text="Generated Code Preview", padding="10")
        preview_frame.grid(row=5, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        self.code_text = scrolledtext.ScrolledText(preview_frame, width=100, height=20, 
                                                   font=('Consolas', 9))
        self.code_text.pack(fill=tk.BOTH, expand=True)
        
        # Status bar
        self.status_var = tk.StringVar(value="Ready")
        status_bar = ttk.Label(main_frame, textvariable=self.status_var, relief=tk.SUNKEN)
        status_bar.grid(row=6, column=0, columnspan=3, sticky=(tk.W, tk.E))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(5, weight=1)
        
    def on_mode_change(self):
        """Handle mode selection change"""
        if self.mode.get() == "lin_to_can":
            # LIN to CAN: need LDF file
            self.ldf_entry.config(state='normal')
            self.ldf_button.config(state='normal')
            self.dbc_entry.config(state='disabled')
            self.dbc_button.config(state='disabled')
            self.status_var.set("Mode: LIN Sniffer → CAN Transmitter (Upload LDF file)")
        else:
            # CAN to LIN: need DBC file
            self.ldf_entry.config(state='disabled')
            self.ldf_button.config(state='disabled')
            self.dbc_entry.config(state='normal')
            self.dbc_button.config(state='normal')
            self.status_var.set("Mode: CAN Receiver → LIN Transmitter (Upload DBC file)")
    
    def browse_ldf(self):
        """Browse for LDF file"""
        filename = filedialog.askopenfilename(
            title="Select LDF File",
            filetypes=[("LIN Description Files", "*.ldf"), ("All Files", "*.*")]
        )
        if filename:
            self.ldf_path.set(filename)
            self.status_var.set(f"LDF file loaded: {Path(filename).name}")
    
    def browse_dbc(self):
        """Browse for DBC file"""
        filename = filedialog.askopenfilename(
            title="Select DBC File",
            filetypes=[("CAN Database Files", "*.dbc"), ("All Files", "*.*")]
        )
        if filename:
            self.dbc_path.set(filename)
            self.status_var.set(f"DBC file loaded: {Path(filename).name}")
    
    def generate_code(self):
        """Generate gateway code"""
        try:
            mode = self.mode.get()
            config = {
                'can_baudrate': self.can_baudrate.get(),
                'lin_baudrate': self.lin_baudrate.get(),
                'can_interval': self.can_interval.get(),
                'can_prescaler': self.calculate_can_prescaler(self.can_baudrate.get())
            }
            
            if mode == "lin_to_can":
                if not self.ldf_path.get():
                    messagebox.showerror("Error", "Please select an LDF file")
                    return
                
                self.status_var.set("Parsing LDF file...")
                self.root.update()
                
                parser = LDFParser(self.ldf_path.get())
                if not parser.parse():
                    messagebox.showerror("Error", "Failed to parse LDF file")
                    return
                
                self.status_var.set("Generating code...")
                self.root.update()
                
                code = CodeGenerator.generate_lin_to_can(parser, config)
                
            else:  # can_to_lin
                if not self.dbc_path.get():
                    messagebox.showerror("Error", "Please select a DBC file")
                    return
                
                self.status_var.set("Parsing DBC file...")
                self.root.update()
                
                parser = DBCParser(self.dbc_path.get())
                if not parser.parse():
                    messagebox.showerror("Error", "Failed to parse DBC file")
                    return
                
                self.status_var.set("Generating code...")
                self.root.update()
                
                code = CodeGenerator.generate_can_to_lin(parser, config)
            
            if code:
                self.code_text.delete(1.0, tk.END)
                self.code_text.insert(1.0, code)
                self.status_var.set(f"Code generated successfully! ({len(code)} characters)")
                messagebox.showinfo("Success", "Code generated successfully!\nReview the code and click 'Save main.cpp'")
            else:
                messagebox.showerror("Error", "Failed to generate code")
                
        except Exception as e:
            messagebox.showerror("Error", f"An error occurred:\n{str(e)}")
            self.status_var.set("Error during code generation")
    
    def save_code(self):
        """Save generated code to file"""
        code = self.code_text.get(1.0, tk.END).strip()
        if not code:
            messagebox.showwarning("Warning", "No code to save. Generate code first.")
            return
        
        filename = filedialog.asksaveasfilename(
            title="Save main.cpp",
            defaultextension=".cpp",
            filetypes=[("C++ Source", "*.cpp"), ("All Files", "*.*")],
            initialfile="main.cpp"
        )
        
        if filename:
            try:
                with open(filename, 'w', encoding='utf-8') as f:
                    f.write(code)
                self.status_var.set(f"Code saved to: {filename}")
                messagebox.showinfo("Success", f"Code saved successfully!\n\n{filename}\n\nCopy this file to your PlatformIO project's src/ folder.")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save file:\n{str(e)}")
    
    def clear_all(self):
        """Clear all inputs and outputs"""
        self.ldf_path.set("")
        self.dbc_path.set("")
        self.code_text.delete(1.0, tk.END)
        self.status_var.set("Cleared - Ready for new configuration")
    
    def calculate_can_prescaler(self, baudrate):
        """Calculate CAN prescaler for STM32F446RE (APB1 = 45MHz)"""
        apb1_freq = 45000000  # 45 MHz
        prescaler_map = {
            1000000: 5,
            500000: 9,
            250000: 18,
            125000: 36
        }
        return prescaler_map.get(baudrate, 9)

def main():
    root = tk.Tk()
    app = GatewayConfigGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()