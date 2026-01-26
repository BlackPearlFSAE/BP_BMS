#!/usr/bin/env python3
"""
BP17 BMS CAN Monitor
====================
Real-time monitoring GUI for Battery Management System via Serial/CAN interface.
Displays cell voltages, temperatures, balancing status, and fault codes.

Protocol: Extended CAN ID (29-bit)
  BCU:  0x180000XX
  BMU:  0x18[Prio][Module]00[Msg]

Usage:
  python bms_monitor.py [--port /dev/ttyUSB0] [--baud 115200] [--demo]
"""

import tkinter as tk
from tkinter import ttk, messagebox
import struct
import threading
import time
import argparse
import random
from dataclasses import dataclass, field
from typing import Optional, Dict, List
from collections import deque

# Try importing serial, provide fallback for demo mode
try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("Warning: pyserial not installed. Running in demo mode only.")


# =============================================================================
# CAN Protocol Definitions (Extended 29-bit IDs)
# =============================================================================

class CANProtocol:
    """Extended CAN ID structure: 0x18[Prio][Module]00[Msg]"""
    
    # Base prefix for BMS
    BMS_PREFIX = 0x18000000
    
    # Priority levels (lower = higher priority in CAN arbitration)
    PRIO_BCU = 0x0        # BCU commands (highest)
    PRIO_FAULT = 0x1      # Fault messages
    PRIO_DATA = 0x2       # Normal data
    
    # Message types
    MSG_STATUS = 0x01     # Status + Balancing + dV + Temps
    MSG_CELLS_LO = 0x02   # Cell 1-8 voltages
    MSG_CELLS_HI = 0x03   # Cell 9-10 voltages
    MSG_FAULT_1 = 0x01    # OV/LV faults (with PRIO_FAULT)
    MSG_FAULT_2 = 0x02    # Temp/dV faults (with PRIO_FAULT)
    
    # BCU Command ID
    BCU_COMMAND = 0x180000FF
    
    @staticmethod
    def build_id(priority: int, module: int, msg: int) -> int:
        """Build extended CAN ID from components."""
        return 0x18000000 | (priority << 20) | (module << 16) | msg
    
    @staticmethod
    def parse_id(can_id: int) -> tuple:
        """Parse extended CAN ID into (priority, module, msg)."""
        if (can_id & 0xFF000000) != 0x18000000:
            return None, None, None
        priority = (can_id >> 20) & 0xF
        module = (can_id >> 16) & 0xF
        msg = can_id & 0xFF
        return priority, module, msg


# =============================================================================
# Data Structures
# =============================================================================

@dataclass
class BMUData:
    """Data container for a single Battery Management Unit."""
    module_id: int
    
    # Cell voltages (10 cells per module)
    cell_voltages: List[float] = field(default_factory=lambda: [0.0] * 10)
    
    # Status
    charging_mode: bool = False
    balancing_cells: int = 0  # 10-bit bitmap
    delta_v: float = 0.0      # Voltage difference
    
    # Temperatures
    temp_sensor_1: float = 0.0
    temp_sensor_2: float = 0.0
    
    # Fault flags (10-bit bitmaps each)
    ov_warning: int = 0
    ov_critical: int = 0
    lv_warning: int = 0
    lv_critical: int = 0
    ot_warning: int = 0
    ot_critical: int = 0
    odv_warning: int = 0
    odv_critical: int = 0
    
    # Timestamps
    last_update: float = 0.0
    last_fault_update: float = 0.0
    
    def get_min_cell_voltage(self) -> float:
        return min(self.cell_voltages) if self.cell_voltages else 0.0
    
    def get_max_cell_voltage(self) -> float:
        return max(self.cell_voltages) if self.cell_voltages else 0.0
    
    def get_total_voltage(self) -> float:
        return sum(self.cell_voltages)
    
    def has_any_fault(self) -> bool:
        return any([
            self.ov_warning, self.ov_critical,
            self.lv_warning, self.lv_critical,
            self.ot_warning, self.ot_critical,
            self.odv_warning, self.odv_critical
        ])
    
    def has_critical_fault(self) -> bool:
        return any([
            self.ov_critical, self.lv_critical,
            self.ot_critical, self.odv_critical
        ])


# =============================================================================
# Serial Frame Parser
# =============================================================================

class CANFrameParser:
    """
    Parse incoming serial data into CAN frames.
    
    Expected format (configurable):
    - ASCII: "18210001#AABBCCDDEEFF0011\n"  (ID#DATA)
    - Binary: [0x18][0x21][0x00][0x01][DLC][DATA...]
    """
    
    def __init__(self, format_type='ascii'):
        self.format_type = format_type
        self.buffer = bytearray()
    
    def feed(self, data: bytes) -> List[tuple]:
        """Feed raw bytes, return list of (can_id, data_bytes) tuples."""
        frames = []
        self.buffer.extend(data)
        
        if self.format_type == 'ascii':
            frames = self._parse_ascii()
        else:
            frames = self._parse_binary()
        
        return frames
    
    def _parse_ascii(self) -> List[tuple]:
        """Parse ASCII format: ID#HEXDATA\n"""
        frames = []
        
        while b'\n' in self.buffer:
            line_end = self.buffer.index(b'\n')
            line = self.buffer[:line_end].decode('ascii', errors='ignore').strip()
            self.buffer = self.buffer[line_end + 1:]
            
            if '#' in line:
                try:
                    id_str, data_str = line.split('#')
                    can_id = int(id_str, 16)
                    data = bytes.fromhex(data_str)
                    frames.append((can_id, data))
                except (ValueError, IndexError):
                    pass
        
        return frames
    
    def _parse_binary(self) -> List[tuple]:
        """Parse binary format: [ID:4][DLC:1][DATA:DLC]"""
        frames = []
        
        while len(self.buffer) >= 5:  # Minimum: 4 byte ID + 1 byte DLC
            can_id = struct.unpack('>I', self.buffer[:4])[0]
            dlc = self.buffer[4]
            
            if dlc > 8:
                self.buffer.pop(0)  # Invalid DLC, shift buffer
                continue
            
            if len(self.buffer) < 5 + dlc:
                break  # Wait for more data
            
            data = bytes(self.buffer[5:5 + dlc])
            self.buffer = self.buffer[5 + dlc:]
            frames.append((can_id, data))
        
        return frames


# =============================================================================
# BMS Data Decoder
# =============================================================================

class BMSDecoder:
    """Decode CAN frames into BMU data structures."""
    
    def __init__(self, num_modules: int = 8):
        self.modules: Dict[int, BMUData] = {
            i: BMUData(module_id=i) for i in range(1, num_modules + 1)
        }
    
    def decode_frame(self, can_id: int, data: bytes) -> Optional[int]:
        """
        Decode a CAN frame and update the corresponding BMU.
        Returns the module ID if successfully decoded, None otherwise.
        """
        priority, module, msg = CANProtocol.parse_id(can_id)
        
        if module is None or module < 1 or module > 8:
            return None
        
        bmu = self.modules.get(module)
        if bmu is None:
            return None
        
        now = time.time()
        
        if priority == CANProtocol.PRIO_DATA:
            if msg == CANProtocol.MSG_STATUS:
                self._decode_status(bmu, data)
                bmu.last_update = now
            elif msg == CANProtocol.MSG_CELLS_LO:
                self._decode_cells_lo(bmu, data)
                bmu.last_update = now
            elif msg == CANProtocol.MSG_CELLS_HI:
                self._decode_cells_hi(bmu, data)
                bmu.last_update = now
        
        elif priority == CANProtocol.PRIO_FAULT:
            if msg == CANProtocol.MSG_FAULT_1:
                self._decode_fault_1(bmu, data)
                bmu.last_fault_update = now
            elif msg == CANProtocol.MSG_FAULT_2:
                self._decode_fault_2(bmu, data)
                bmu.last_fault_update = now
        
        return module
    
    def _decode_status(self, bmu: BMUData, data: bytes):
        """Decode MSG1: Status + Balancing + dV + Temps"""
        if len(data) < 6:
            return
        
        # Byte 0: Charging mode flag
        bmu.charging_mode = bool(data[0] & 0x01)
        
        # Byte 1-2: Balancing cells (10-bit bitmap, MSB first)
        bmu.balancing_cells = ((data[1] << 8) | data[2]) >> 6  # Top 10 bits
        
        # Byte 3: Delta V (0-0.2V, factor 0.1)
        bmu.delta_v = data[3] * 0.1
        
        # Byte 4-5: Temp sensors (offset 2, factor 0.0125)
        bmu.temp_sensor_1 = 2.0 + data[4] * 0.0125
        bmu.temp_sensor_2 = 2.0 + data[5] * 0.0125
    
    def _decode_cells_lo(self, bmu: BMUData, data: bytes):
        """Decode MSG2: Cell 1-8 voltages"""
        for i in range(min(8, len(data))):
            # Raw value to voltage: factor 0.02
            bmu.cell_voltages[i] = data[i] * 0.02
    
    def _decode_cells_hi(self, bmu: BMUData, data: bytes):
        """Decode MSG3: Cell 9-10 voltages"""
        for i in range(min(2, len(data))):
            bmu.cell_voltages[8 + i] = data[i] * 0.02
    
    def _decode_fault_1(self, bmu: BMUData, data: bytes):
        """Decode Fault MSG1: OV/LV warnings and critical"""
        if len(data) < 8:
            return
        
        # Each fault is 10-bit bitmap across 2 bytes
        bmu.ov_warning = ((data[0] << 8) | data[1]) >> 6
        bmu.ov_critical = ((data[2] << 8) | data[3]) >> 6
        bmu.lv_warning = ((data[4] << 8) | data[5]) >> 6
        bmu.lv_critical = ((data[6] << 8) | data[7]) >> 6
    
    def _decode_fault_2(self, bmu: BMUData, data: bytes):
        """Decode Fault MSG2: Temp/dV warnings and critical"""
        if len(data) < 8:
            return
        
        bmu.ot_warning = ((data[0] << 8) | data[1]) >> 6
        bmu.ot_critical = ((data[2] << 8) | data[3]) >> 6
        bmu.odv_warning = ((data[4] << 8) | data[5]) >> 6
        bmu.odv_critical = ((data[6] << 8) | data[7]) >> 6


# =============================================================================
# Demo Data Generator
# =============================================================================

class DemoDataGenerator:
    """Generate realistic demo data for testing without hardware."""
    
    def __init__(self, decoder: BMSDecoder):
        self.decoder = decoder
        self.running = False
        self.thread = None
        self.time_offset = 0
    
    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._generate_loop, daemon=True)
        self.thread.start()
    
    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
    
    def _generate_loop(self):
        while self.running:
            self.time_offset += 0.1
            
            for module_id in range(1, 9):
                bmu = self.decoder.modules[module_id]
                
                # Generate cell voltages (3.2V - 4.2V range with slight variation)
                base_voltage = 3.7 + 0.3 * (module_id / 8)
                for i in range(10):
                    noise = random.gauss(0, 0.02)
                    drift = 0.1 * (i / 10) * (1 + 0.5 * (module_id % 3))
                    bmu.cell_voltages[i] = max(3.2, min(4.2, base_voltage + drift + noise))
                
                # Balancing simulation (random cells)
                if random.random() < 0.1:
                    bmu.balancing_cells = random.randint(0, 0x3FF)
                
                # Delta V
                bmu.delta_v = max(bmu.cell_voltages) - min(bmu.cell_voltages)
                
                # Temperatures (simulate heating during operation)
                base_temp = 2.5 + 0.3 * (self.time_offset % 60) / 60
                bmu.temp_sensor_1 = base_temp + random.gauss(0, 0.05)
                bmu.temp_sensor_2 = base_temp + 0.1 + random.gauss(0, 0.05)
                
                # Occasional faults for demo
                if random.random() < 0.02:
                    fault_cell = 1 << random.randint(0, 9)
                    if random.random() < 0.7:
                        bmu.ov_warning = fault_cell
                    else:
                        bmu.ov_warning = 0
                
                bmu.last_update = time.time()
            
            time.sleep(0.5)  # 500ms update rate


# =============================================================================
# GUI Application
# =============================================================================

class BMSMonitorGUI:
    """Main GUI application for BMS monitoring."""
    
    # Color scheme - Industrial/Technical aesthetic
    COLORS = {
        'bg_dark': '#0a0e14',
        'bg_panel': '#141a22',
        'bg_cell': '#1a2230',
        'accent': '#00ff9f',
        'accent_dim': '#00aa6f',
        'warning': '#ffaa00',
        'critical': '#ff3366',
        'text': '#e0e0e0',
        'text_dim': '#707080',
        'border': '#2a3a4a',
        'cell_ok': '#00cc7a',
        'cell_low': '#ff9933',
        'cell_high': '#ff3366',
        'balance_on': '#00aaff',
        'temp_cold': '#00aaff',
        'temp_hot': '#ff6633',
    }
    
    def __init__(self, root: tk.Tk, decoder: BMSDecoder, serial_port: Optional[str] = None):
        self.root = root
        self.decoder = decoder
        self.serial_port = serial_port
        self.serial_conn = None
        self.parser = CANFrameParser(format_type='ascii')
        self.running = False
        self.demo_mode = False
        self.demo_generator = None
        
        # Message log
        self.message_log = deque(maxlen=100)
        
        self._setup_window()
        self._create_widgets()
        self._start_update_loop()
    
    def _setup_window(self):
        """Configure main window."""
        self.root.title("BP17 BMS Monitor")
        self.root.geometry("1400x900")
        self.root.configure(bg=self.COLORS['bg_dark'])
        self.root.minsize(1200, 700)
        
        # Configure grid weights
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(1, weight=1)
    
    def _create_widgets(self):
        """Create all GUI widgets."""
        self._create_header()
        self._create_main_area()
        self._create_status_bar()
    
    def _create_header(self):
        """Create header with connection controls."""
        header = tk.Frame(self.root, bg=self.COLORS['bg_panel'], height=60)
        header.grid(row=0, column=0, sticky='ew', padx=10, pady=(10, 5))
        header.grid_propagate(False)
        
        # Title
        title = tk.Label(
            header,
            text="⚡ BP17 BMS MONITOR",
            font=('Consolas', 18, 'bold'),
            fg=self.COLORS['accent'],
            bg=self.COLORS['bg_panel']
        )
        title.pack(side='left', padx=20, pady=10)
        
        # Connection frame
        conn_frame = tk.Frame(header, bg=self.COLORS['bg_panel'])
        conn_frame.pack(side='right', padx=20, pady=10)
        
        # Port selection
        tk.Label(
            conn_frame,
            text="PORT:",
            font=('Consolas', 10),
            fg=self.COLORS['text_dim'],
            bg=self.COLORS['bg_panel']
        ).pack(side='left', padx=(0, 5))
        
        self.port_var = tk.StringVar(value=self.serial_port or "")
        self.port_combo = ttk.Combobox(
            conn_frame,
            textvariable=self.port_var,
            width=15,
            font=('Consolas', 10)
        )
        self.port_combo.pack(side='left', padx=5)
        self._refresh_ports()
        
        # Refresh button
        self.refresh_btn = tk.Button(
            conn_frame,
            text="⟳",
            font=('Consolas', 12),
            fg=self.COLORS['text'],
            bg=self.COLORS['bg_cell'],
            activebackground=self.COLORS['accent_dim'],
            bd=0,
            width=3,
            command=self._refresh_ports
        )
        self.refresh_btn.pack(side='left', padx=5)
        
        # Connect button
        self.connect_btn = tk.Button(
            conn_frame,
            text="CONNECT",
            font=('Consolas', 10, 'bold'),
            fg=self.COLORS['bg_dark'],
            bg=self.COLORS['accent'],
            activebackground=self.COLORS['accent_dim'],
            bd=0,
            width=10,
            command=self._toggle_connection
        )
        self.connect_btn.pack(side='left', padx=10)
        
        # Demo button
        self.demo_btn = tk.Button(
            conn_frame,
            text="DEMO",
            font=('Consolas', 10),
            fg=self.COLORS['text'],
            bg=self.COLORS['bg_cell'],
            activebackground=self.COLORS['warning'],
            bd=0,
            width=8,
            command=self._toggle_demo
        )
        self.demo_btn.pack(side='left', padx=5)
    
    def _create_main_area(self):
        """Create main monitoring area."""
        main = tk.Frame(self.root, bg=self.COLORS['bg_dark'])
        main.grid(row=1, column=0, sticky='nsew', padx=10, pady=5)
        main.grid_columnconfigure(0, weight=3)
        main.grid_columnconfigure(1, weight=1)
        main.grid_rowconfigure(0, weight=1)
        
        # Left: Module grid
        self._create_module_grid(main)
        
        # Right: Summary panel
        self._create_summary_panel(main)
    
    def _create_module_grid(self, parent):
        """Create grid of BMU module displays."""
        grid_frame = tk.Frame(parent, bg=self.COLORS['bg_dark'])
        grid_frame.grid(row=0, column=0, sticky='nsew', padx=(0, 10))
        
        # Configure 2x4 grid
        for i in range(2):
            grid_frame.grid_columnconfigure(i, weight=1)
        for i in range(4):
            grid_frame.grid_rowconfigure(i, weight=1)
        
        self.module_frames = {}
        self.cell_labels = {}
        self.bitmap_canvases = {}
        self.temp_labels = {}
        self.voltage_labels = {}
        
        for idx, module_id in enumerate(range(1, 9)):
            row = idx // 2
            col = idx % 2
            self._create_module_panel(grid_frame, module_id, row, col)
    
    def _create_module_panel(self, parent, module_id: int, row: int, col: int):
        """Create a single BMU module panel."""
        panel = tk.Frame(
            parent,
            bg=self.COLORS['bg_panel'],
            highlightbackground=self.COLORS['border'],
            highlightthickness=1
        )
        panel.grid(row=row, column=col, sticky='nsew', padx=5, pady=5)
        self.module_frames[module_id] = panel
        
        # Header
        header = tk.Frame(panel, bg=self.COLORS['bg_panel'])
        header.pack(fill='x', padx=10, pady=(10, 5))
        
        tk.Label(
            header,
            text=f"BMU {module_id}",
            font=('Consolas', 12, 'bold'),
            fg=self.COLORS['accent'],
            bg=self.COLORS['bg_panel']
        ).pack(side='left')
        
        # Status indicator
        self.voltage_labels[module_id] = tk.Label(
            header,
            text="-- V",
            font=('Consolas', 11),
            fg=self.COLORS['text'],
            bg=self.COLORS['bg_panel']
        )
        self.voltage_labels[module_id].pack(side='right')
        
        # Cell voltages grid (2 rows x 5 cols)
        cells_frame = tk.Frame(panel, bg=self.COLORS['bg_panel'])
        cells_frame.pack(fill='x', padx=10, pady=5)
        
        self.cell_labels[module_id] = []
        for i in range(10):
            row_idx = i // 5
            col_idx = i % 5
            
            cell_frame = tk.Frame(cells_frame, bg=self.COLORS['bg_cell'], width=55, height=40)
            cell_frame.grid(row=row_idx, column=col_idx, padx=2, pady=2)
            cell_frame.grid_propagate(False)
            
            tk.Label(
                cell_frame,
                text=f"C{i+1}",
                font=('Consolas', 7),
                fg=self.COLORS['text_dim'],
                bg=self.COLORS['bg_cell']
            ).pack(anchor='nw', padx=3, pady=(2, 0))
            
            voltage_lbl = tk.Label(
                cell_frame,
                text="-.--",
                font=('Consolas', 10, 'bold'),
                fg=self.COLORS['text'],
                bg=self.COLORS['bg_cell']
            )
            voltage_lbl.pack(expand=True)
            self.cell_labels[module_id].append(voltage_lbl)
        
        # Bitmap displays
        bitmap_frame = tk.Frame(panel, bg=self.COLORS['bg_panel'])
        bitmap_frame.pack(fill='x', padx=10, pady=5)
        
        self.bitmap_canvases[module_id] = {}
        
        # Balancing bitmap
        bal_frame = tk.Frame(bitmap_frame, bg=self.COLORS['bg_panel'])
        bal_frame.pack(side='left', padx=(0, 15))
        
        tk.Label(
            bal_frame,
            text="BAL",
            font=('Consolas', 8),
            fg=self.COLORS['text_dim'],
            bg=self.COLORS['bg_panel']
        ).pack(anchor='w')
        
        bal_canvas = tk.Canvas(
            bal_frame,
            width=100,
            height=12,
            bg=self.COLORS['bg_cell'],
            highlightthickness=0
        )
        bal_canvas.pack()
        self.bitmap_canvases[module_id]['balance'] = bal_canvas
        
        # Fault bitmap
        fault_frame = tk.Frame(bitmap_frame, bg=self.COLORS['bg_panel'])
        fault_frame.pack(side='left')
        
        tk.Label(
            fault_frame,
            text="FLT",
            font=('Consolas', 8),
            fg=self.COLORS['text_dim'],
            bg=self.COLORS['bg_panel']
        ).pack(anchor='w')
        
        fault_canvas = tk.Canvas(
            fault_frame,
            width=100,
            height=12,
            bg=self.COLORS['bg_cell'],
            highlightthickness=0
        )
        fault_canvas.pack()
        self.bitmap_canvases[module_id]['fault'] = fault_canvas
        
        # Temperature display
        temp_frame = tk.Frame(panel, bg=self.COLORS['bg_panel'])
        temp_frame.pack(fill='x', padx=10, pady=(5, 10))
        
        self.temp_labels[module_id] = []
        for i in range(2):
            tk.Label(
                temp_frame,
                text=f"T{i+1}:",
                font=('Consolas', 9),
                fg=self.COLORS['text_dim'],
                bg=self.COLORS['bg_panel']
            ).pack(side='left')
            
            temp_lbl = tk.Label(
                temp_frame,
                text="--.-V",
                font=('Consolas', 9),
                fg=self.COLORS['text'],
                bg=self.COLORS['bg_panel']
            )
            temp_lbl.pack(side='left', padx=(0, 15))
            self.temp_labels[module_id].append(temp_lbl)
        
        # Delta V
        tk.Label(
            temp_frame,
            text="ΔV:",
            font=('Consolas', 9),
            fg=self.COLORS['text_dim'],
            bg=self.COLORS['bg_panel']
        ).pack(side='left')
        
        dv_lbl = tk.Label(
            temp_frame,
            text="--mV",
            font=('Consolas', 9),
            fg=self.COLORS['text'],
            bg=self.COLORS['bg_panel']
        )
        dv_lbl.pack(side='left')
        self.temp_labels[module_id].append(dv_lbl)  # Index 2 = delta V
    
    def _create_summary_panel(self, parent):
        """Create summary panel on the right."""
        summary = tk.Frame(parent, bg=self.COLORS['bg_panel'])
        summary.grid(row=0, column=1, sticky='nsew')
        
        # Pack summary
        tk.Label(
            summary,
            text="PACK SUMMARY",
            font=('Consolas', 12, 'bold'),
            fg=self.COLORS['accent'],
            bg=self.COLORS['bg_panel']
        ).pack(pady=(15, 10))
        
        # Summary stats
        stats_frame = tk.Frame(summary, bg=self.COLORS['bg_panel'])
        stats_frame.pack(fill='x', padx=15, pady=10)
        
        self.summary_labels = {}
        
        stats = [
            ('total_v', 'Total Voltage', '--- V'),
            ('min_cell', 'Min Cell', '-.-- V'),
            ('max_cell', 'Max Cell', '-.-- V'),
            ('delta_v', 'Pack ΔV', '--- mV'),
            ('max_temp', 'Max Temp', '-.-- V'),
            ('faults', 'Active Faults', '0'),
        ]
        
        for key, label, default in stats:
            row = tk.Frame(stats_frame, bg=self.COLORS['bg_panel'])
            row.pack(fill='x', pady=3)
            
            tk.Label(
                row,
                text=label,
                font=('Consolas', 10),
                fg=self.COLORS['text_dim'],
                bg=self.COLORS['bg_panel']
            ).pack(side='left')
            
            val_label = tk.Label(
                row,
                text=default,
                font=('Consolas', 11, 'bold'),
                fg=self.COLORS['text'],
                bg=self.COLORS['bg_panel']
            )
            val_label.pack(side='right')
            self.summary_labels[key] = val_label
        
        # Separator
        tk.Frame(summary, bg=self.COLORS['border'], height=1).pack(fill='x', padx=15, pady=15)
        
        # Message log
        tk.Label(
            summary,
            text="CAN LOG",
            font=('Consolas', 10, 'bold'),
            fg=self.COLORS['text_dim'],
            bg=self.COLORS['bg_panel']
        ).pack(pady=(0, 5))
        
        log_frame = tk.Frame(summary, bg=self.COLORS['bg_cell'])
        log_frame.pack(fill='both', expand=True, padx=15, pady=(0, 15))
        
        self.log_text = tk.Text(
            log_frame,
            font=('Consolas', 8),
            fg=self.COLORS['text'],
            bg=self.COLORS['bg_cell'],
            height=15,
            width=30,
            state='disabled',
            wrap='none'
        )
        self.log_text.pack(fill='both', expand=True, padx=2, pady=2)
    
    def _create_status_bar(self):
        """Create status bar at bottom."""
        status = tk.Frame(self.root, bg=self.COLORS['bg_panel'], height=30)
        status.grid(row=2, column=0, sticky='ew', padx=10, pady=(5, 10))
        status.grid_propagate(False)
        
        self.status_label = tk.Label(
            status,
            text="Disconnected",
            font=('Consolas', 9),
            fg=self.COLORS['text_dim'],
            bg=self.COLORS['bg_panel']
        )
        self.status_label.pack(side='left', padx=10, pady=5)
        
        self.rx_label = tk.Label(
            status,
            text="RX: 0",
            font=('Consolas', 9),
            fg=self.COLORS['text_dim'],
            bg=self.COLORS['bg_panel']
        )
        self.rx_label.pack(side='right', padx=10, pady=5)
    
    def _refresh_ports(self):
        """Refresh available serial ports."""
        if SERIAL_AVAILABLE:
            ports = [p.device for p in serial.tools.list_ports.comports()]
        else:
            ports = ['/dev/ttyUSB0', '/dev/ttyACM0', 'COM3']  # Dummy for demo
        
        self.port_combo['values'] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])
    
    def _toggle_connection(self):
        """Connect or disconnect serial."""
        if self.running:
            self._disconnect()
        else:
            self._connect()
    
    def _connect(self):
        """Establish serial connection."""
        if not SERIAL_AVAILABLE:
            messagebox.showwarning("Warning", "pyserial not installed. Use DEMO mode.")
            return
        
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Error", "Select a serial port")
            return
        
        try:
            self.serial_conn = serial.Serial(port, 115200, timeout=0.1)
            self.running = True
            self.connect_btn.configure(text="DISCONNECT", bg=self.COLORS['critical'])
            self.status_label.configure(text=f"Connected: {port}", fg=self.COLORS['accent'])
            
            # Start read thread
            self.read_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
            self.read_thread.start()
            
        except serial.SerialException as e:
            messagebox.showerror("Connection Error", str(e))
    
    def _disconnect(self):
        """Close serial connection."""
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()
            self.serial_conn = None
        
        self.connect_btn.configure(text="CONNECT", bg=self.COLORS['accent'])
        self.status_label.configure(text="Disconnected", fg=self.COLORS['text_dim'])
    
    def _toggle_demo(self):
        """Toggle demo mode."""
        if self.demo_mode:
            self._stop_demo()
        else:
            self._start_demo()
    
    def _start_demo(self):
        """Start demo data generation."""
        if self.running:
            self._disconnect()
        
        self.demo_mode = True
        self.demo_generator = DemoDataGenerator(self.decoder)
        self.demo_generator.start()
        
        self.demo_btn.configure(text="STOP DEMO", bg=self.COLORS['warning'])
        self.status_label.configure(text="Demo Mode", fg=self.COLORS['warning'])
    
    def _stop_demo(self):
        """Stop demo data generation."""
        self.demo_mode = False
        if self.demo_generator:
            self.demo_generator.stop()
            self.demo_generator = None
        
        self.demo_btn.configure(text="DEMO", bg=self.COLORS['bg_cell'])
        self.status_label.configure(text="Disconnected", fg=self.COLORS['text_dim'])
    
    def _serial_read_loop(self):
        """Background thread for reading serial data."""
        rx_count = 0
        
        while self.running and self.serial_conn:
            try:
                data = self.serial_conn.read(256)
                if data:
                    frames = self.parser.feed(data)
                    for can_id, payload in frames:
                        self.decoder.decode_frame(can_id, payload)
                        rx_count += 1
                        
                        # Log message
                        log_msg = f"{can_id:08X}#{payload.hex().upper()}"
                        self.message_log.append(log_msg)
                    
                    # Update RX counter in main thread
                    self.root.after(0, lambda c=rx_count: self.rx_label.configure(text=f"RX: {c}"))
            
            except serial.SerialException:
                break
    
    def _start_update_loop(self):
        """Start GUI update loop."""
        self._update_display()
        self.root.after(100, self._start_update_loop)  # 10 Hz refresh
    
    def _update_display(self):
        """Update all display elements."""
        all_voltages = []
        all_temps = []
        fault_count = 0
        
        for module_id, bmu in self.decoder.modules.items():
            # Update cell voltages
            for i, voltage in enumerate(bmu.cell_voltages):
                label = self.cell_labels[module_id][i]
                
                if voltage > 0:
                    label.configure(text=f"{voltage:.2f}")
                    all_voltages.append(voltage)
                    
                    # Color based on voltage level
                    if voltage >= 4.15:
                        label.configure(fg=self.COLORS['cell_high'])
                    elif voltage <= 3.3:
                        label.configure(fg=self.COLORS['cell_low'])
                    else:
                        label.configure(fg=self.COLORS['cell_ok'])
                else:
                    label.configure(text="-.--", fg=self.COLORS['text_dim'])
            
            # Update module total voltage
            total_v = bmu.get_total_voltage()
            if total_v > 0:
                self.voltage_labels[module_id].configure(text=f"{total_v:.1f}V")
            
            # Update temperatures
            if bmu.temp_sensor_1 > 0:
                self.temp_labels[module_id][0].configure(text=f"{bmu.temp_sensor_1:.2f}V")
                all_temps.append(bmu.temp_sensor_1)
            if bmu.temp_sensor_2 > 0:
                self.temp_labels[module_id][1].configure(text=f"{bmu.temp_sensor_2:.2f}V")
                all_temps.append(bmu.temp_sensor_2)
            
            # Update delta V
            if bmu.delta_v > 0:
                dv_mv = bmu.delta_v * 1000
                self.temp_labels[module_id][2].configure(text=f"{dv_mv:.0f}mV")
            
            # Update balancing bitmap
            self._draw_bitmap(
                self.bitmap_canvases[module_id]['balance'],
                bmu.balancing_cells,
                self.COLORS['balance_on']
            )
            
            # Update fault bitmap (combine all faults)
            combined_faults = (
                bmu.ov_warning | bmu.ov_critical |
                bmu.lv_warning | bmu.lv_critical |
                bmu.ot_warning | bmu.ot_critical |
                bmu.odv_warning | bmu.odv_critical
            )
            self._draw_bitmap(
                self.bitmap_canvases[module_id]['fault'],
                combined_faults,
                self.COLORS['critical'] if bmu.has_critical_fault() else self.COLORS['warning']
            )
            
            if bmu.has_any_fault():
                fault_count += 1
            
            # Update panel border based on status
            panel = self.module_frames[module_id]
            if bmu.has_critical_fault():
                panel.configure(highlightbackground=self.COLORS['critical'])
            elif bmu.has_any_fault():
                panel.configure(highlightbackground=self.COLORS['warning'])
            elif time.time() - bmu.last_update < 2.0:
                panel.configure(highlightbackground=self.COLORS['accent'])
            else:
                panel.configure(highlightbackground=self.COLORS['border'])
        
        # Update summary
        if all_voltages:
            self.summary_labels['total_v'].configure(text=f"{sum(all_voltages):.1f} V")
            self.summary_labels['min_cell'].configure(text=f"{min(all_voltages):.2f} V")
            self.summary_labels['max_cell'].configure(text=f"{max(all_voltages):.2f} V")
            delta = (max(all_voltages) - min(all_voltages)) * 1000
            self.summary_labels['delta_v'].configure(text=f"{delta:.0f} mV")
        
        if all_temps:
            self.summary_labels['max_temp'].configure(text=f"{max(all_temps):.2f} V")
        
        self.summary_labels['faults'].configure(
            text=str(fault_count),
            fg=self.COLORS['critical'] if fault_count > 0 else self.COLORS['accent']
        )
        
        # Update log
        self._update_log()
    
    def _draw_bitmap(self, canvas: tk.Canvas, value: int, color: str):
        """Draw a 10-bit bitmap on canvas."""
        canvas.delete('all')
        
        cell_width = 9
        cell_height = 10
        padding = 1
        
        for i in range(10):
            x = i * (cell_width + padding) + 2
            y = 1
            
            bit_set = bool(value & (1 << (9 - i)))  # MSB first
            fill = color if bit_set else self.COLORS['bg_dark']
            
            canvas.create_rectangle(
                x, y, x + cell_width, y + cell_height,
                fill=fill,
                outline=self.COLORS['border']
            )
    
    def _update_log(self):
        """Update CAN message log."""
        if not self.message_log:
            return
        
        self.log_text.configure(state='normal')
        self.log_text.delete('1.0', tk.END)
        
        for msg in list(self.message_log)[-20:]:  # Show last 20
            self.log_text.insert(tk.END, msg + '\n')
        
        self.log_text.see(tk.END)
        self.log_text.configure(state='disabled')
    
    def on_closing(self):
        """Clean up on window close."""
        self._disconnect()
        self._stop_demo()
        self.root.destroy()


# =============================================================================
# Main Entry Point
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description='BP17 BMS CAN Monitor')
    parser.add_argument('--port', '-p', type=str, help='Serial port (e.g., /dev/ttyUSB0)')
    parser.add_argument('--baud', '-b', type=int, default=115200, help='Baud rate')
    parser.add_argument('--demo', '-d', action='store_true', help='Start in demo mode')
    args = parser.parse_args()
    
    # Create data structures
    decoder = BMSDecoder(num_modules=8)
    
    # Create GUI
    root = tk.Tk()
    app = BMSMonitorGUI(root, decoder, serial_port=args.port)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    # Auto-start demo if requested
    if args.demo:
        root.after(500, app._start_demo)
    
    root.mainloop()


if __name__ == '__main__':
    main()