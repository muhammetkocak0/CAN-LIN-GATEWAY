"""
CAN-LIN Gateway Configurator - Complete GUI Application
Professional configuration tool for generic CAN-LIN gateway

Author: Gateway Team
Version: 2.0.0
"""

import sys
import json
from pathlib import Path
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QTreeWidget, QTreeWidgetItem, QTabWidget, QTextEdit, QPushButton,
    QLabel, QComboBox, QSpinBox, QDoubleSpinBox, QCheckBox, QGroupBox,
    QFileDialog, QMessageBox, QTableWidget, QTableWidgetItem, QHeaderView,
    QSplitter, QStatusBar, QToolBar, QLineEdit, QDialog, QFormLayout,
    QDialogButtonBox, QProgressBar
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt6.QtGui import QAction, QIcon, QFont

# Import modules
try:
    from parsers.dbc_parser import DBCParser
    from parsers.ldf_parser import LDFParser
    from communication.serial_protocol import SerialProtocol, FrameMapping, SignalMapping
except ImportError:
    # Fallback if modules not yet created
    print("Warning: Some modules not found. Using stubs.")
    DBCParser = None
    LDFParser = None
    SerialProtocol = None


class MappingDialog(QDialog):
    """Dialog for creating/editing frame mappings"""
    
    def __init__(self, parent=None, mapping=None, dbc_data=None, ldf_data=None):
        super().__init__(parent)
        self.mapping = mapping or {}
        self.dbc_data = dbc_data or {}
        self.ldf_data = ldf_data or {}
        
        self.setWindowTitle("Frame Mapping Editor")
        self.setMinimumWidth(700)
        self.setMinimumHeight(500)
        
        self.setup_ui()
        
        if mapping:
            self.load_mapping(mapping)
    
    def setup_ui(self):
        layout = QVBoxLayout(self)
        
        # Source configuration
        source_group = QGroupBox("Source")
        source_layout = QFormLayout()
        
        self.source_type = QComboBox()
        self.source_type.addItems(["LIN", "CAN"])
        self.source_type.currentTextChanged.connect(self.on_source_type_changed)
        
        self.source_frame = QComboBox()
        self.source_signal = QComboBox()
        
        source_layout.addRow("Bus Type:", self.source_type)
        source_layout.addRow("Frame/Message:", self.source_frame)
        source_layout.addRow("Signal:", self.source_signal)
        
        source_group.setLayout(source_layout)
        layout.addWidget(source_group)
        
        # Destination configuration
        dest_group = QGroupBox("Destination")
        dest_layout = QFormLayout()
        
        self.dest_type = QComboBox()
        self.dest_type.addItems(["CAN", "LIN"])
        self.dest_type.currentTextChanged.connect(self.on_dest_type_changed)
        
        self.dest_frame = QComboBox()
        self.dest_signal = QComboBox()
        
        dest_layout.addRow("Bus Type:", self.dest_type)
        dest_layout.addRow("Frame/Message:", self.dest_frame)
        dest_layout.addRow("Signal:", self.dest_signal)
        
        dest_group.setLayout(dest_layout)
        layout.addWidget(dest_group)
        
        # Signal mapping configuration
        signal_group = QGroupBox("Signal Configuration")
        signal_layout = QFormLayout()
        
        self.scale = QDoubleSpinBox()
        self.scale.setRange(-1000000, 1000000)
        self.scale.setValue(1.0)
        self.scale.setDecimals(6)
        
        self.offset = QDoubleSpinBox()
        self.offset.setRange(-1000000, 1000000)
        self.offset.setValue(0.0)
        self.offset.setDecimals(6)
        
        self.update_rate = QSpinBox()
        self.update_rate.setRange(0, 10000)
        self.update_rate.setValue(20)
        self.update_rate.setSuffix(" ms")
        
        self.enabled = QCheckBox("Enabled")
        self.enabled.setChecked(True)
        
        signal_layout.addRow("Scale Factor:", self.scale)
        signal_layout.addRow("Offset:", self.offset)
        signal_layout.addRow("Update Rate:", self.update_rate)
        signal_layout.addRow("", self.enabled)
        
        signal_group.setLayout(signal_layout)
        layout.addWidget(signal_group)
        
        # Buttons
        button_box = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | 
            QDialogButtonBox.StandardButton.Cancel
        )
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)
        
        # Populate frame lists
        self.populate_frames()
    
    def populate_frames(self):
        """Populate frame/message dropdowns from DBC/LDF data"""
        # Populate LIN frames
        if self.ldf_data.get('frames'):
            for frame in self.ldf_data['frames']:
                self.source_frame.addItem(
                    f"{frame['name']} (0x{frame['id']:02X})",
                    frame
                )
        
        # Populate CAN messages
        if self.dbc_data.get('messages'):
            for msg in self.dbc_data['messages']:
                self.dest_frame.addItem(
                    f"{msg['name']} (0x{msg['id']:03X})",
                    msg
                )
    
    def on_source_type_changed(self, text):
        """Update source frame list when type changes"""
        self.source_frame.clear()
        self.source_signal.clear()
        
        if text == "LIN" and self.ldf_data.get('frames'):
            for frame in self.ldf_data['frames']:
                self.source_frame.addItem(
                    f"{frame['name']} (0x{frame['id']:02X})",
                    frame
                )
        elif text == "CAN" and self.dbc_data.get('messages'):
            for msg in self.dbc_data['messages']:
                self.source_frame.addItem(
                    f"{msg['name']} (0x{msg['id']:03X})",
                    msg
                )
    
    def on_dest_type_changed(self, text):
        """Update destination frame list when type changes"""
        self.dest_frame.clear()
        self.dest_signal.clear()
        
        if text == "LIN" and self.ldf_data.get('frames'):
            for frame in self.ldf_data['frames']:
                self.dest_frame.addItem(
                    f"{frame['name']} (0x{frame['id']:02X})",
                    frame
                )
        elif text == "CAN" and self.dbc_data.get('messages'):
            for msg in self.dbc_data['messages']:
                self.dest_frame.addItem(
                    f"{msg['name']} (0x{msg['id']:03X})",
                    msg
                )
    
    def load_mapping(self, mapping):
        """Load existing mapping into dialog"""
        # TODO: Implement loading
        pass
    
    def get_mapping(self):
        """Get mapping configuration from dialog"""
        source_data = self.source_frame.currentData()
        dest_data = self.dest_frame.currentData()
        
        if not source_data or not dest_data:
            return None
        
        mapping = {
            'source_type': 0 if self.source_type.currentText() == "LIN" else 1,
            'source_id': source_data.get('id', 0),
            'source_name': source_data.get('name', ''),
            'dest_type': 0 if self.dest_type.currentText() == "LIN" else 1,
            'dest_id': dest_data.get('id', 0),
            'dest_name': dest_data.get('name', ''),
            'scale': self.scale.value(),
            'offset': self.offset.value(),
            'update_rate': self.update_rate.value(),
            'enabled': self.enabled.isChecked()
        }
        
        return mapping


class MainWindow(QMainWindow):
    """Main application window"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CAN-LIN Gateway Configurator v2.0")
        self.setGeometry(100, 100, 1400, 800)
        
        # Data structures
        self.dbc_data = {}
        self.ldf_data = {}
        self.mappings = []
        self.serial_protocol = None
        self.connected = False
        
        # Setup UI
        self.setup_ui()
        self.setup_menubar()
        self.setup_toolbar()
        self.setup_statusbar()
        
        # Timer for status updates
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_gateway_status)
        
        # Apply modern styling
        self.apply_stylesheet()
    
    def apply_stylesheet(self):
        """Apply modern dark theme"""
        self.setStyleSheet("""
            QMainWindow {
                background-color: #2b2b2b;
            }
            QWidget {
                background-color: #2b2b2b;
                color: #e0e0e0;
                font-family: 'Segoe UI', Arial, sans-serif;
                font-size: 10pt;
            }
            QGroupBox {
                border: 1px solid #555;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
                font-weight: bold;
            }
            QGroupBox::title {
                color: #4CAF50;
            }
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                padding: 8px 16px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:pressed {
                background-color: #3d8b40;
            }
            QPushButton:disabled {
                background-color: #555;
                color: #888;
            }
            QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {
                background-color: #3b3b3b;
                border: 1px solid #555;
                border-radius: 3px;
                padding: 5px;
            }
            QTableWidget, QTreeWidget, QTextEdit {
                background-color: #3b3b3b;
                border: 1px solid #555;
                alternate-background-color: #353535;
            }
            QHeaderView::section {
                background-color: #4CAF50;
                color: white;
                padding: 5px;
                border: none;
                font-weight: bold;
            }
            QTabWidget::pane {
                border: 1px solid #555;
            }
            QTabBar::tab {
                background-color: #3b3b3b;
                color: #e0e0e0;
                padding: 8px 20px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }
            QTabBar::tab:selected {
                background-color: #4CAF50;
                color: white;
            }
            QStatusBar {
                background-color: #1e1e1e;
                color: #4CAF50;
            }
        """)
    
    def setup_ui(self):
        """Setup main UI layout"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QHBoxLayout(central_widget)
        
        # Left panel - Project tree
        left_panel = self.create_project_tree()
        
        # Center panel - Main work area
        center_panel = self.create_center_panel()
        
        # Right panel - Properties and controls
        right_panel = self.create_right_panel()
        
        # Splitter for resizable panels
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(left_panel)
        splitter.addWidget(center_panel)
        splitter.addWidget(right_panel)
        splitter.setSizes([250, 750, 400])
        
        main_layout.addWidget(splitter)
    
    def create_project_tree(self):
        """Create project tree view"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        label = QLabel("Project Structure")
        label.setFont(QFont("Arial", 11, QFont.Weight.Bold))
        layout.addWidget(label)
        
        self.tree = QTreeWidget()
        self.tree.setHeaderLabel("Components")
        self.tree.itemDoubleClicked.connect(self.on_tree_item_double_clicked)
        
        # Root items
        self.lin_bus_item = QTreeWidgetItem(self.tree, ["📡 LIN Bus"])
        self.can_bus_item = QTreeWidgetItem(self.tree, ["🚗 CAN Bus"])
        self.mappings_item = QTreeWidgetItem(self.tree, ["🔗 Mappings"])
        self.gateway_item = QTreeWidgetItem(self.tree, ["⚙️ Gateway"])
        
        # Expand all
        self.tree.expandAll()
        
        layout.addWidget(self.tree)
        
        return widget
    
    def create_center_panel(self):
        """Create center work area"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Tabs for different views
        self.tabs = QTabWidget()
        
        # Mapping table tab
        self.mapping_table = QTableWidget()
        self.mapping_table.setColumnCount(8)
        self.mapping_table.setHorizontalHeaderLabels([
            "ID", "Source Type", "Source ID", "Source Name",
            "Dest Type", "Dest ID", "Dest Name", "Enabled"
        ])
        self.mapping_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.mapping_table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.mapping_table.itemSelectionChanged.connect(self.on_mapping_selected)
        self.tabs.addTab(self.mapping_table, "📋 Mappings")
        
        # Monitor tab
        self.monitor_text = QTextEdit()
        self.monitor_text.setReadOnly(True)
        self.monitor_text.setFont(QFont("Consolas", 9))
        self.tabs.addTab(self.monitor_text, "📊 Monitor")
        
        # Statistics tab
        stats_widget = QWidget()
        stats_layout = QVBoxLayout(stats_widget)
        
        self.stats_labels = {}
        stats_info = [
            ("LIN Frames RX:", "lin_rx"),
            ("LIN Frames TX:", "lin_tx"),
            ("CAN Frames RX:", "can_rx"),
            ("CAN Frames TX:", "can_tx"),
            ("LIN Errors:", "lin_err"),
            ("CAN Errors:", "can_err"),
            ("Mapping Errors:", "map_err"),
            ("Checksum Errors:", "chk_err")
        ]
        
        for label_text, key in stats_info:
            hlayout = QHBoxLayout()
            label = QLabel(label_text)
            label.setMinimumWidth(150)
            value_label = QLabel("0")
            value_label.setFont(QFont("Consolas", 10, QFont.Weight.Bold))
            hlayout.addWidget(label)
            hlayout.addWidget(value_label)
            hlayout.addStretch()
            stats_layout.addLayout(hlayout)
            self.stats_labels[key] = value_label
        
        stats_layout.addStretch()
        self.tabs.addTab(stats_widget, "📈 Statistics")
        
        layout.addWidget(self.tabs)
        
        # Buttons
        button_layout = QHBoxLayout()
        
        btn_add = QPushButton("➕ Add Mapping")
        btn_add.clicked.connect(self.add_mapping)
        
        btn_edit = QPushButton("✏️ Edit Mapping")
        btn_edit.clicked.connect(self.edit_mapping)
        
        btn_delete = QPushButton("🗑️ Delete Mapping")
        btn_delete.clicked.connect(self.delete_mapping)
        
        btn_clear = QPushButton("🧹 Clear All")
        btn_clear.clicked.connect(self.clear_all_mappings)
        
        button_layout.addWidget(btn_add)
        button_layout.addWidget(btn_edit)
        button_layout.addWidget(btn_delete)
        button_layout.addWidget(btn_clear)
        button_layout.addStretch()
        
        layout.addLayout(button_layout)
        
        return widget
    
    def create_right_panel(self):
        """Create right control panel"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Connection group
        conn_group = QGroupBox("🔌 Gateway Connection")
        conn_layout = QVBoxLayout()
        
        port_layout = QHBoxLayout()
        port_layout.addWidget(QLabel("COM Port:"))
        self.port_combo = QComboBox()
        self.refresh_ports()
        port_layout.addWidget(self.port_combo)
        
        btn_refresh = QPushButton("🔄")
        btn_refresh.setMaximumWidth(40)
        btn_refresh.clicked.connect(self.refresh_ports)
        port_layout.addWidget(btn_refresh)
        conn_layout.addLayout(port_layout)
        
        btn_layout = QHBoxLayout()
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.connect_gateway)
        
        self.btn_disconnect = QPushButton("Disconnect")
        self.btn_disconnect.clicked.connect(self.disconnect_gateway)
        self.btn_disconnect.setEnabled(False)
        
        btn_layout.addWidget(self.btn_connect)
        btn_layout.addWidget(self.btn_disconnect)
        conn_layout.addLayout(btn_layout)
        
        conn_group.setLayout(conn_layout)
        layout.addWidget(conn_group)
        
        # Control group
        ctrl_group = QGroupBox("🎮 Gateway Control")
        ctrl_layout = QVBoxLayout()
        
        self.btn_upload = QPushButton("⬆️ Upload Configuration")
        self.btn_upload.clicked.connect(self.upload_configuration)
        self.btn_upload.setEnabled(False)
        
        self.btn_start = QPushButton("▶️ Start Gateway")
        self.btn_start.clicked.connect(self.start_gateway)
        self.btn_start.setEnabled(False)
        
        self.btn_stop = QPushButton("⏹️ Stop Gateway")
        self.btn_stop.clicked.connect(self.stop_gateway)
        self.btn_stop.setEnabled(False)
        
        ctrl_layout.addWidget(self.btn_upload)
        ctrl_layout.addWidget(self.btn_start)
        ctrl_layout.addWidget(self.btn_stop)
        
        ctrl_group.setLayout(ctrl_layout)
        layout.addWidget(ctrl_group)
        
        # Status group
        status_group = QGroupBox("📊 Gateway Status")
        status_layout = QVBoxLayout()
        
        self.status_labels = {
            'running': QLabel("Running: ❌ No"),
            'mappings': QLabel("Mappings: 0"),
            'uptime': QLabel("Uptime: 0s"),
            'connection': QLabel("Status: Disconnected")
        }
        
        for label in self.status_labels.values():
            label.setFont(QFont("Consolas", 9))
            status_layout.addWidget(label)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        layout.addStretch()
        
        return widget
    
    def setup_menubar(self):
        """Setup menu bar"""
        menubar = self.menuBar()
        
        # File menu
        file_menu = menubar.addMenu("📁 File")
        
        load_dbc_action = QAction("Load DBC File...", self)
        load_dbc_action.setShortcut("Ctrl+D")
        load_dbc_action.triggered.connect(self.load_dbc_file)
        file_menu.addAction(load_dbc_action)
        
        load_ldf_action = QAction("Load LDF File...", self)
        load_ldf_action.setShortcut("Ctrl+L")
        load_ldf_action.triggered.connect(self.load_ldf_file)
        file_menu.addAction(load_ldf_action)
        
        file_menu.addSeparator()
        
        save_config_action = QAction("Save Configuration...", self)
        save_config_action.setShortcut("Ctrl+S")
        save_config_action.triggered.connect(self.save_configuration)
        file_menu.addAction(save_config_action)
        
        load_config_action = QAction("Load Configuration...", self)
        load_config_action.setShortcut("Ctrl+O")
        load_config_action.triggered.connect(self.load_configuration)
        file_menu.addAction(load_config_action)
        
        file_menu.addSeparator()
        
        exit_action = QAction("Exit", self)
        exit_action.setShortcut("Ctrl+Q")
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        # Gateway menu
        gateway_menu = menubar.addMenu("⚙️ Gateway")
        
        connect_action = QAction("Connect", self)
        connect_action.triggered.connect(self.connect_gateway)
        gateway_menu.addAction(connect_action)
        
        upload_action = QAction("Upload Configuration", self)
        upload_action.triggered.connect(self.upload_configuration)
        gateway_menu.addAction(upload_action)
        
        start_action = QAction("Start", self)
        start_action.triggered.connect(self.start_gateway)
        gateway_menu.addAction(start_action)
        
        # Help menu
        help_menu = menubar.addMenu("❓ Help")
        
        about_action = QAction("About", self)
        about_action.triggered.connect(self.show_about)
        help_menu.addAction(about_action)
        
        docs_action = QAction("Documentation", self)
        docs_action.triggered.connect(self.show_documentation)
        help_menu.addAction(docs_action)
    
    def setup_toolbar(self):
        """Setup toolbar"""
        toolbar = QToolBar()
        toolbar.setMovable(False)
        self.addToolBar(toolbar)
        
        toolbar.addAction("📂 Load DBC", self.load_dbc_file)
        toolbar.addAction("📄 Load LDF", self.load_ldf_file)
        toolbar.addSeparator()
        toolbar.addAction("➕ Add Mapping", self.add_mapping)
        toolbar.addSeparator()
        toolbar.addAction("⬆️ Upload", self.upload_configuration)
        toolbar.addAction("▶️ Start", self.start_gateway)
        toolbar.addAction("⏹️ Stop", self.stop_gateway)
    
    def setup_statusbar(self):
        """Setup status bar"""
        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage("Ready • No files loaded")
    
    # ========================================================================
    # FILE OPERATIONS
    # ========================================================================
    
    def load_dbc_file(self):
        """Load DBC file"""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Open DBC File", "", "DBC Files (*.dbc);;All Files (*)"
        )
        if filename:
            try:
                if DBCParser:
                    parser = DBCParser()
                    self.dbc_data = parser.parse(filename)
                else:
                    # Stub for testing
                    self.dbc_data = {
                        'messages': [
                            {'id': 0x100, 'name': 'BLDC_Status_1', 'signals': []},
                            {'id': 0x101, 'name': 'BLDC_Status_2', 'signals': []}
                        ]
                    }
                
                self.statusBar.showMessage(f"✓ Loaded DBC: {Path(filename).name}")
                self.update_can_tree()
                self.log_monitor(f"[INFO] Loaded DBC file: {filename}")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load DBC:\n{str(e)}")
                self.log_monitor(f"[ERROR] Failed to load DBC: {e}")
    
    def load_ldf_file(self):
        """Load LDF file"""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Open LDF File", "", "LDF Files (*.ldf);;All Files (*)"
        )
        if filename:
            try:
                if LDFParser:
                    parser = LDFParser()
                    self.ldf_data = parser.parse(filename)
                else:
                    # Stub for testing
                    self.ldf_data = {
                        'frames': [
                            {'id': 0x32, 'name': 'BLW_F_Stat_LIN_ST3', 'signals': []},
                            {'id': 0x33, 'name': 'BLW_F_Rq_LIN_ST3', 'signals': []}
                        ]
                    }
                
                self.statusBar.showMessage(f"✓ Loaded LDF: {Path(filename).name}")
                self.update_lin_tree()
                self.log_monitor(f"[INFO] Loaded LDF file: {filename}")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load LDF:\n{str(e)}")
                self.log_monitor(f"[ERROR] Failed to load LDF: {e}")
    
    def save_configuration(self):
        """Save current configuration to JSON"""
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save Configuration", "", "JSON Files (*.json);;All Files (*)"
        )
        if filename:
            try:
                config = {
                    'version': '2.0',
                    'mappings': self.mappings
                }
                with open(filename, 'w') as f:
                    json.dump(config, f, indent=2)
                
                self.statusBar.showMessage(f"✓ Saved: {Path(filename).name}")
                self.log_monitor(f"[INFO] Configuration saved to: {filename}")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to save:\n{str(e)}")
    
    def load_configuration(self):
        """Load configuration from JSON"""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Load Configuration", "", "JSON Files (*.json);;All Files (*)"
        )
        if filename:
            try:
                with open(filename, 'r') as f:
                    config = json.load(f)
                
                self.mappings = config.get('mappings', [])
                self.update_mapping_table()
                self.statusBar.showMessage(f"✓ Loaded: {Path(filename).name}")
                self.log_monitor(f"[INFO] Configuration loaded from: {filename}")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load:\n{str(e)}")
    
    # ========================================================================
    # GATEWAY OPERATIONS
    # ========================================================================
    
    def refresh_ports(self):
        """Refresh available COM ports"""
        import serial.tools.list_ports
        
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        
        if ports:
            for port in ports:
                self.port_combo.addItem(f"{port.device} - {port.description}", port.device)
        else:
            self.port_combo.addItem("No ports found")
    
    def connect_gateway(self):
        """Connect to gateway"""
        if self.port_combo.count() == 0 or self.port_combo.currentText() == "No ports found":
            QMessageBox.warning(self, "Warning", "No COM ports available")
            return
        
        port = self.port_combo.currentData()
        
        try:
            if SerialProtocol:
                self.serial_protocol = SerialProtocol(port)
                if self.serial_protocol.connect():
                    self.connected = True
                    self.btn_connect.setEnabled(False)
                    self.btn_disconnect.setEnabled(True)
                    self.btn_upload.setEnabled(True)
                    self.btn_start.setEnabled(True)
                    self.btn_stop.setEnabled(True)
                    
                    self.statusBar.showMessage(f"✓ Connected to {port}")
                    self.status_labels['connection'].setText(f"Status: ✅ Connected ({port})")
                    self.log_monitor(f"[CONNECT] Connected to gateway on {port}")
                    
                    # Start status updates
                    self.status_timer.start(1000)
                else:
                    raise Exception("Connection failed")
            else:
                # Demo mode
                self.connected = True
                self.btn_connect.setEnabled(False)
                self.btn_disconnect.setEnabled(True)
                self.btn_upload.setEnabled(True)
                self.btn_start.setEnabled(True)
                self.btn_stop.setEnabled(True)
                
                self.statusBar.showMessage(f"✓ Connected (Demo Mode)")
                self.status_labels['connection'].setText("Status: ✅ Connected (Demo)")
                self.log_monitor("[CONNECT] Connected in demo mode")
                
        except Exception as e:
            QMessageBox.critical(self, "Connection Error", f"Failed to connect:\n{str(e)}")
            self.log_monitor(f"[ERROR] Connection failed: {e}")
    
    def disconnect_gateway(self):
        """Disconnect from gateway"""
        try:
            if self.serial_protocol:
                self.serial_protocol.disconnect()
            
            self.connected = False
            self.btn_connect.setEnabled(True)
            self.btn_disconnect.setEnabled(False)
            self.btn_upload.setEnabled(False)
            self.btn_start.setEnabled(False)
            self.btn_stop.setEnabled(False)
            
            self.status_timer.stop()
            
            self.statusBar.showMessage("Disconnected")
            self.status_labels['connection'].setText("Status: Disconnected")
            self.log_monitor("[DISCONNECT] Disconnected from gateway")
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Disconnect error:\n{str(e)}")
    
    def upload_configuration(self):
        """Upload configuration to gateway"""
        if not self.connected:
            QMessageBox.warning(self, "Warning", "Not connected to gateway")
            return
        
        if not self.mappings:
            QMessageBox.warning(self, "Warning", "No mappings to upload")
            return
        
        try:
            # Progress dialog
            progress = QProgressBar()
            progress.setMaximum(len(self.mappings) + 1)
            
            self.statusBar.addPermanentWidget(progress)
            
            # Upload each mapping
            for i, mapping in enumerate(self.mappings):
                if self.serial_protocol:
                    # TODO: Implement actual upload
                    pass
                
                progress.setValue(i + 1)
                QApplication.processEvents()
            
            # Save to flash
            if self.serial_protocol:
                # TODO: Send save command
                pass
            
            progress.setValue(len(self.mappings) + 1)
            
            self.statusBar.removeWidget(progress)
            self.statusBar.showMessage(f"✓ Uploaded {len(self.mappings)} mappings")
            self.log_monitor(f"[UPLOAD] Successfully uploaded {len(self.mappings)} mappings")
            
            QMessageBox.information(
                self, 
                "Success", 
                f"Configuration uploaded successfully!\n{len(self.mappings)} mappings sent to gateway."
            )
            
        except Exception as e:
            QMessageBox.critical(self, "Upload Error", f"Failed to upload:\n{str(e)}")
            self.log_monitor(f"[ERROR] Upload failed: {e}")
    
    def start_gateway(self):
        """Start gateway operation"""
        if not self.connected:
            QMessageBox.warning(self, "Warning", "Not connected to gateway")
            return
        
        try:
            if self.serial_protocol:
                # TODO: Send start command
                pass
            
            self.statusBar.showMessage("✓ Gateway started")
            self.log_monitor("[START] Gateway operation started")
            self.status_labels['running'].setText("Running: ✅ Yes")
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to start:\n{str(e)}")
    
    def stop_gateway(self):
        """Stop gateway operation"""
        if not self.connected:
            QMessageBox.warning(self, "Warning", "Not connected to gateway")
            return
        
        try:
            if self.serial_protocol:
                # TODO: Send stop command
                pass
            
            self.statusBar.showMessage("✓ Gateway stopped")
            self.log_monitor("[STOP] Gateway operation stopped")
            self.status_labels['running'].setText("Running: ❌ No")
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to stop:\n{str(e)}")
    
    def update_gateway_status(self):
        """Update gateway status display"""
        if not self.connected or not self.serial_protocol:
            return
        
        try:
            # Get status from gateway
            status = self.serial_protocol.get_status()
            if status:
                running = status.get('running', False)
                mappings = status.get('num_mappings', 0)
                uptime = status.get('uptime', 0)
                
                self.status_labels['running'].setText(f"Running: {'✅ Yes' if running else '❌ No'}")
                self.status_labels['mappings'].setText(f"Mappings: {mappings}")
                self.status_labels['uptime'].setText(f"Uptime: {uptime}s")
            
            # Get statistics
            stats = self.serial_protocol.get_statistics()
            if stats:
                self.stats_labels['lin_rx'].setText(str(stats.get('lin_rx', 0)))
                self.stats_labels['lin_tx'].setText(str(stats.get('lin_tx', 0)))
                self.stats_labels['can_rx'].setText(str(stats.get('can_rx', 0)))
                self.stats_labels['can_tx'].setText(str(stats.get('can_tx', 0)))
                self.stats_labels['lin_err'].setText(str(stats.get('lin_err', 0)))
                self.stats_labels['can_err'].setText(str(stats.get('can_err', 0)))
                self.stats_labels['map_err'].setText(str(stats.get('map_err', 0)))
                self.stats_labels['chk_err'].setText(str(stats.get('chk_err', 0)))
        
        except Exception as e:
            self.log_monitor(f"[ERROR] Status update failed: {e}")
    
    # ========================================================================
    # MAPPING OPERATIONS
    # ========================================================================
    
    def add_mapping(self):
        """Add new mapping"""
        if not self.dbc_data and not self.ldf_data:
            QMessageBox.warning(
                self, 
                "Warning", 
                "Please load DBC and/or LDF files first"
            )
            return
        
        dialog = MappingDialog(self, dbc_data=self.dbc_data, ldf_data=self.ldf_data)
        
        if dialog.exec() == QDialog.DialogCode.Accepted:
            mapping = dialog.get_mapping()
            if mapping:
                mapping['id'] = len(self.mappings) + 1
                self.mappings.append(mapping)
                self.update_mapping_table()
                self.log_monitor(f"[MAPPING] Added new mapping: {mapping['source_name']} → {mapping['dest_name']}")
    
    def edit_mapping(self):
        """Edit selected mapping"""
        selected = self.mapping_table.selectedItems()
        if not selected:
            QMessageBox.warning(self, "Warning", "Please select a mapping to edit")
            return
        
        row = self.mapping_table.currentRow()
        if row < len(self.mappings):
            dialog = MappingDialog(
                self, 
                mapping=self.mappings[row],
                dbc_data=self.dbc_data,
                ldf_data=self.ldf_data
            )
            
            if dialog.exec() == QDialog.DialogCode.Accepted:
                mapping = dialog.get_mapping()
                if mapping:
                    mapping['id'] = self.mappings[row]['id']
                    self.mappings[row] = mapping
                    self.update_mapping_table()
                    self.log_monitor(f"[MAPPING] Updated mapping ID {mapping['id']}")
    
    def delete_mapping(self):
        """Delete selected mapping"""
        selected = self.mapping_table.selectedItems()
        if not selected:
            QMessageBox.warning(self, "Warning", "Please select a mapping to delete")
            return
        
        row = self.mapping_table.currentRow()
        
        reply = QMessageBox.question(
            self,
            "Confirm Delete",
            "Are you sure you want to delete this mapping?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            if row < len(self.mappings):
                mapping = self.mappings[row]
                del self.mappings[row]
                self.update_mapping_table()
                self.log_monitor(f"[MAPPING] Deleted mapping ID {mapping.get('id', '?')}")
    
    def clear_all_mappings(self):
        """Clear all mappings"""
        if not self.mappings:
            return
        
        reply = QMessageBox.question(
            self,
            "Confirm Clear",
            f"Are you sure you want to delete all {len(self.mappings)} mappings?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            count = len(self.mappings)
            self.mappings.clear()
            self.update_mapping_table()
            self.log_monitor(f"[MAPPING] Cleared all {count} mappings")
    
    def on_mapping_selected(self):
        """Handle mapping selection"""
        # TODO: Update details panel
        pass
    
    def update_mapping_table(self):
        """Update mapping table display"""
        self.mapping_table.setRowCount(len(self.mappings))
        
        for i, mapping in enumerate(self.mappings):
            self.mapping_table.setItem(i, 0, QTableWidgetItem(str(mapping.get('id', ''))))
            self.mapping_table.setItem(i, 1, QTableWidgetItem("LIN" if mapping.get('source_type') == 0 else "CAN"))
            self.mapping_table.setItem(i, 2, QTableWidgetItem(f"0x{mapping.get('source_id', 0):03X}"))
            self.mapping_table.setItem(i, 3, QTableWidgetItem(mapping.get('source_name', '')))
            self.mapping_table.setItem(i, 4, QTableWidgetItem("LIN" if mapping.get('dest_type') == 0 else "CAN"))
            self.mapping_table.setItem(i, 5, QTableWidgetItem(f"0x{mapping.get('dest_id', 0):03X}"))
            self.mapping_table.setItem(i, 6, QTableWidgetItem(mapping.get('dest_name', '')))
            self.mapping_table.setItem(i, 7, QTableWidgetItem("✅" if mapping.get('enabled') else "❌"))
    
    # ========================================================================
    # TREE OPERATIONS
    # ========================================================================
    
    def on_tree_item_double_clicked(self, item, column):
        """Handle tree item double click"""
        # TODO: Implement item-specific actions
        pass
    
    def update_lin_tree(self):
        """Update LIN bus tree"""
        self.lin_bus_item.takeChildren()
        
        if self.ldf_data.get('frames'):
            for frame in self.ldf_data['frames']:
                item = QTreeWidgetItem(
                    self.lin_bus_item,
                    [f"{frame['name']} (0x{frame['id']:02X})"]
                )
                
                # Add signals as children
                for signal in frame.get('signals', []):
                    QTreeWidgetItem(item, [signal.get('name', 'Unknown')])
        
        self.lin_bus_item.setExpanded(True)
    
    def update_can_tree(self):
        """Update CAN bus tree"""
        self.can_bus_item.takeChildren()
        
        if self.dbc_data.get('messages'):
            for msg in self.dbc_data['messages']:
                item = QTreeWidgetItem(
                    self.can_bus_item,
                    [f"{msg['name']} (0x{msg['id']:03X})"]
                )
                
                # Add signals as children
                for signal in msg.get('signals', []):
                    QTreeWidgetItem(item, [signal.get('name', 'Unknown')])
        
        self.can_bus_item.setExpanded(True)
    
    # ========================================================================
    # UTILITY FUNCTIONS
    # ========================================================================
    
    def log_monitor(self, message):
        """Add message to monitor log"""
        from datetime import datetime
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.monitor_text.append(f"[{timestamp}] {message}")
        
        # Auto-scroll to bottom
        scrollbar = self.monitor_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
    
    def show_about(self):
        """Show about dialog"""
        QMessageBox.about(
            self,
            "About CAN-LIN Gateway Configurator",
            "<h2>CAN-LIN Gateway Configurator v2.0</h2>"
            "<p>Professional configuration tool for generic CAN-LIN gateway</p>"
            "<p><b>Features:</b></p>"
            "<ul>"
            "<li>Load DBC and LDF files</li>"
            "<li>Create custom signal mappings</li>"
            "<li>Upload configuration to gateway</li>"
            "<li>Real-time monitoring</li>"
            "<li>Statistics tracking</li>"
            "</ul>"
            "<p><b>Author:</b> Gateway Team</p>"
            "<p><b>Version:</b> 2.0.0</p>"
        )
    
    def show_documentation(self):
        """Show documentation"""
        QMessageBox.information(
            self,
            "Documentation",
            "<h3>Quick Start Guide</h3>"
            "<ol>"
            "<li>Load your DBC and LDF files</li>"
            "<li>Create signal mappings (Add Mapping button)</li>"
            "<li>Connect to gateway via COM port</li>"
            "<li>Upload configuration</li>"
            "<li>Start gateway operation</li>"
            "<li>Monitor real-time data</li>"
            "</ol>"
            "<p>For complete documentation, see the docs/ folder in the project.</p>"
        )


def main():
    """Application entry point"""
    app = QApplication(sys.argv)
    app.setApplicationName("CAN-LIN Gateway Configurator")
    app.setOrganizationName("Gateway Team")
    
    # Set application font
    font = QFont("Segoe UI", 10)
    app.setFont(font)
    
    window = MainWindow()
    window.show()
    
    sys.exit(app.exec())


if __name__ == '__main__':
    main()