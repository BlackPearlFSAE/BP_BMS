#!/usr/bin/env python3
"""
BP16B BMS CAN Monitor
=====================
Real-time monitoring GUI for Battery Management System via CAN bus.
Implements BCU-equivalent fault monitoring logic from bcu.cpp.

Fault conditions (matching BCU):
1. Communication timeout - no CAN messages for DISCONNECT_TIMEOUT
2. Module disconnection - any BMU not responding
3. Critical faults - OV/LV/OT/DV critical flags from any module

Usage:
    python Monitor.py --demo                                    # Demo mode
    python Monitor.py --can slcan --channel /dev/ttyACM0        # USB adapter
    python Monitor.py --can pcan --channel PCAN_USBBUS1         # PCAN
    python Monitor.py --can socketcan --channel can0            # Linux SocketCAN
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import argparse
import random
from dataclasses import dataclass, field
from typing import Optional, Dict, List, Callable
from enum import Enum

# python-can import
try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False
    print("Warning: python-can not installed. Use: pip install python-can")

# =============================================================================
# Runtime Configuration
# =============================================================================
class Config:
    """Runtime configuration - matches BCU parameters from ams_data_util.h"""
    CELL_NUM = 10
    MODULE_NUM = 8
    CAN_BITRATE = 250000
    DISCONNECT_TIMEOUT = 1.5  # seconds (DISCONNENCTION_TIMEOUT in BCU)

    # Voltage thresholds (from bcu.cpp defaults)
    CELL_V_HIGH = 4.15  # VMAX_CELL
    CELL_V_LOW = 3.3    # VMIN_CELL
    TEMP_MAX = 60       # TEMP_MAX_CELL
    DV_MAX = 0.5        # DVMAX (500mV)

# =============================================================================
# Theme
# =============================================================================
C = {
    'bg': '#1d1d1d', 'panel': '#2d2d2d', 'cell': '#353535', 'header': '#252525',
    'accent': '#ff7f00', 'accent_dim': '#cc6600', 'ok': '#33cc33',
    'warn': '#ff9500', 'crit': '#ff3333', 'text': '#e0e0e0', 'dim': '#808080',
    'border': '#454545', 'disconn': '#555555', 'bal': '#00aaff',
}

FAULT_TYPES = [
    ('ov_warning', 'OV Warn', False), ('ov_critical', 'OV Crit', True),
    ('lv_warning', 'LV Warn', False), ('lv_critical', 'LV Crit', True),
    ('ot_warning', 'OT Warn', False), ('ot_critical', 'OT Crit', True),
    ('odv_warning', 'DV Warn', False), ('odv_critical', 'DV Crit', True),
]

# =============================================================================
# AMS Fault Reason (matches BCU logic)
# =============================================================================
class FaultReason(Enum):
    NONE = "OK"
    NO_COMMUNICATION = "NO CAN RX"
    MODULE_DISCONNECTED = "BMU OFFLINE"
    OVERVOLT_CRITICAL = "OV CRIT"
    LOWVOLT_CRITICAL = "LV CRIT"
    OVERTEMP_CRITICAL = "OT CRIT"
    OVERDIV_CRITICAL = "DV CRIT"

# =============================================================================
# Data Model
# =============================================================================
@dataclass
class BMUData:
    """BMU module data - matches BMUdata struct in ams_data_util.h"""
    module_id: int
    cell_voltages: List[float] = field(default_factory=list)
    balancing_cells: int = 0
    need_balance: bool = False
    delta_v: float = 0.0
    temps: List[float] = field(default_factory=lambda: [0.0, 0.0])
    ov_warning: int = 0
    ov_critical: int = 0
    lv_warning: int = 0
    lv_critical: int = 0
    ot_warning: int = 0
    ot_critical: int = 0
    odv_warning: int = 0
    odv_critical: int = 0
    last_update: float = 0.0

    def __post_init__(self):
        if not self.cell_voltages:
            self.cell_voltages = [0.0] * Config.CELL_NUM

    def total_v(self) -> float:
        return sum(self.cell_voltages)

    def is_connected(self) -> bool:
        if self.last_update == 0:
            return False
        return (time.time() - self.last_update) < Config.DISCONNECT_TIMEOUT

    def has_warning(self) -> bool:
        return any([self.ov_warning, self.lv_warning, self.ot_warning, self.odv_warning])

    def has_critical(self) -> bool:
        return any([self.ov_critical, self.lv_critical, self.ot_critical, self.odv_critical])

    def has_fault(self) -> bool:
        return self.has_warning() or self.has_critical()

    def cell_fault(self, attr: str, idx: int) -> bool:
        """Check if specific cell has fault (MSB first like BCU)"""
        bitmap = getattr(self, attr, 0)
        bit_position = 9 - idx  # MSB first
        return bool(bitmap & (1 << bit_position))

# =============================================================================
# BMS Decoder with BCU-equivalent fault logic
# =============================================================================
class BMSDecoder:
    """
    Decodes CAN messages and determines AMS_OK status.
    Implements same fault logic as BCU main loop in bcu.cpp.
    """

    def __init__(self, module_count: int = None):
        self.module_count = module_count or Config.MODULE_NUM
        self.modules: Dict[int, BMUData] = {}
        self._rebuild_modules()

        # AMS status (matches BCU)
        self.ams_ok = False  # Start false until we have valid data
        self.fault_reason = FaultReason.NO_COMMUNICATION
        self.last_rx_time = 0.0

        # Aggregate data
        self.accum_voltage = 0.0
        self.min_cell_v = 0.0
        self.max_cell_v = 0.0
        self.pack_delta_v = 0.0
        self.max_temp = 0.0
        self.connected_count = 0
        self.fault_count = 0

    def _rebuild_modules(self):
        """Rebuild module dict when count changes"""
        self.modules = {i: BMUData(module_id=i) for i in range(1, self.module_count + 1)}

    def set_module_count(self, count: int):
        """Update module count at runtime"""
        if count != self.module_count:
            self.module_count = count
            Config.MODULE_NUM = count
            self._rebuild_modules()

    def decode(self, can_id: int, data: bytes) -> Optional[int]:
        """
        Decode CAN message. Returns module_id if valid, None otherwise.
        CAN ID format: 0x18[Prio:4][Module:4]00[Msg:4]
        """
        # Check extended ID prefix
        if (can_id & 0xFF000000) != 0x18000000:
            return None

        prio = (can_id >> 20) & 0xF
        mod = (can_id >> 16) & 0xF
        msg = can_id & 0xFF

        if mod < 1 or mod > self.module_count:
            return None

        bmu = self.modules[mod]
        now = time.time()
        self.last_rx_time = now

        # Priority 0x02: Data messages (from bmu.cpp sendCellData)
        if prio == 2:
            if msg == 1 and len(data) >= 8:  # MSG1: Status
                bmu.need_balance = bool(data[0])
                bmu.balancing_cells = ((data[1] << 8) | data[2]) & 0x3FF
                bmu.delta_v = data[3] * 0.1  # DV in 100mV units
                # Temperature: 16-bit scaled by 10 (bmu.cpp line 421-426)
                # Temp1: data[4-5], Temp2: data[6-7]
                bmu.temps[0] = ((data[4] << 8) | data[5]) / 10.0
                bmu.temps[1] = ((data[6] << 8) | data[7]) / 10.0
                bmu.last_update = now

            elif msg == 2 and len(data) >= 8:  # MSG2: Cells 1-8
                for i in range(8):
                    bmu.cell_voltages[i] = data[i] * 0.02  # 20mV per bit
                bmu.last_update = now

            elif msg == 3 and len(data) >= 2:  # MSG3: Cells 9-10
                bmu.cell_voltages[8] = data[0] * 0.02
                bmu.cell_voltages[9] = data[1] * 0.02
                bmu.last_update = now

        # Priority 0x01: Fault messages (from bmu.cpp sendFaultData)
        elif prio == 1:
            if msg == 1 and len(data) >= 8:  # Fault MSG1: OV/LV
                bmu.ov_warning = ((data[0] << 8) | data[1]) & 0x3FF
                bmu.ov_critical = ((data[2] << 8) | data[3]) & 0x3FF
                bmu.lv_warning = ((data[4] << 8) | data[5]) & 0x3FF
                bmu.lv_critical = ((data[6] << 8) | data[7]) & 0x3FF
                bmu.last_update = now

            elif msg == 2 and len(data) >= 8:  # Fault MSG2: OT/DV
                bmu.ot_warning = ((data[0] << 8) | data[1]) & 0x3FF
                bmu.ot_critical = ((data[2] << 8) | data[3]) & 0x3FF
                bmu.odv_warning = ((data[4] << 8) | data[5]) & 0x3FF
                bmu.odv_critical = ((data[6] << 8) | data[7]) & 0x3FF
                bmu.last_update = now

        return mod

    def evaluate_ams_status(self) -> tuple:
        """
        Evaluate AMS_OK status using BCU logic from bcu.cpp main loop.
        Returns (ams_ok, fault_reason)
        """
        now = time.time()

        # Check 1: Complete communication loss (bcu.cpp line 210)
        if self.last_rx_time == 0 or (now - self.last_rx_time) > Config.DISCONNECT_TIMEOUT:
            return False, FaultReason.NO_COMMUNICATION

        # Check 2: Any module disconnected (bcu.cpp line 221-240)
        for bmu in self.modules.values():
            if not bmu.is_connected():
                return False, FaultReason.MODULE_DISCONNECTED

        # Check 3: Critical faults (bcu.cpp line 287-288)
        # ACCUMULATOR_Fault = OVER_VOLT_CRIT || LOW_VOLT_CRIT || OVER_TEMP_CRIT || ACCUM_OverDivCritical
        for bmu in self.modules.values():
            if bmu.ov_critical:
                return False, FaultReason.OVERVOLT_CRITICAL
            if bmu.lv_critical:
                return False, FaultReason.LOWVOLT_CRITICAL
            if bmu.ot_critical:
                return False, FaultReason.OVERTEMP_CRITICAL
            if bmu.odv_critical:
                return False, FaultReason.OVERDIV_CRITICAL

        return True, FaultReason.NONE

    def aggregate(self):
        """
        Aggregate pack data and evaluate AMS status.
        Called periodically like BCU's 500ms aggregation loop.
        """
        # Evaluate AMS status first
        self.ams_ok, self.fault_reason = self.evaluate_ams_status()

        # Aggregate statistics
        all_voltages = []
        all_temps = []
        self.accum_voltage = 0.0
        self.connected_count = 0
        self.fault_count = 0

        for bmu in self.modules.values():
            if bmu.is_connected():
                self.connected_count += 1
                self.accum_voltage += bmu.total_v()

                for v in bmu.cell_voltages:
                    if v > 0:
                        all_voltages.append(v)

                for t in bmu.temps:
                    if t > 0:
                        all_temps.append(t)

                if bmu.has_fault():
                    self.fault_count += 1

        if all_voltages:
            self.min_cell_v = min(all_voltages)
            self.max_cell_v = max(all_voltages)
            self.pack_delta_v = self.max_cell_v - self.min_cell_v
        else:
            self.min_cell_v = self.max_cell_v = self.pack_delta_v = 0.0

        self.max_temp = max(all_temps) if all_temps else 0.0

    def reset(self):
        """Reset all data (like BCU resetAllStruct)"""
        self._rebuild_modules()
        self.last_rx_time = 0.0
        self.ams_ok = False
        self.fault_reason = FaultReason.NO_COMMUNICATION

# =============================================================================
# CAN Source
# =============================================================================
class CANSource:
    def __init__(self, decoder: BMSDecoder):
        self.decoder = decoder
        self.bus: Optional['can.Bus'] = None
        self.running = False
        self.rx_count = 0
        self.bitrate = Config.CAN_BITRATE
        self.error_msg = ""

    def connect(self, interface: str, channel: str, bitrate: int = None) -> bool:
        if not CAN_AVAILABLE:
            self.error_msg = "python-can not installed"
            return False

        self.bitrate = bitrate or Config.CAN_BITRATE
        try:
            kwargs = {'interface': interface, 'channel': channel}
            if interface != 'socketcan':
                kwargs['bitrate'] = self.bitrate
            if interface == 'slcan':
                kwargs['ttyBaudrate'] = 115200

            self.bus = can.Bus(**kwargs)
            self.running = True
            self.rx_count = 0
            threading.Thread(target=self._read_loop, daemon=True).start()
            return True
        except Exception as e:
            self.error_msg = str(e)
            print(f"CAN connect error: {e}")
            return False

    def disconnect(self):
        self.running = False
        if self.bus:
            try:
                self.bus.shutdown()
            except:
                pass
            self.bus = None

    def _read_loop(self):
        while self.running and self.bus:
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg and msg.is_extended_id:
                    if self.decoder.decode(msg.arbitration_id, msg.data):
                        self.rx_count += 1
            except Exception:
                pass

# =============================================================================
# Demo Generator
# =============================================================================
class DemoGenerator:
    def __init__(self, decoder: BMSDecoder):
        self.decoder = decoder
        self.running = False

    def start(self):
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self.running = False

    def _loop(self):
        t = 0
        while self.running:
            t += 0.5
            for mid in range(1, self.decoder.module_count + 1):
                if mid not in self.decoder.modules:
                    continue
                bmu = self.decoder.modules[mid]

                # Generate cell voltages
                base = 3.7 + 0.2 * (mid / self.decoder.module_count)
                for i in range(Config.CELL_NUM):
                    bmu.cell_voltages[i] = max(3.2, min(4.2, base + 0.05 * i + random.gauss(0, 0.02)))

                bmu.delta_v = max(bmu.cell_voltages) - min(bmu.cell_voltages)
                # Demo temps: realistic values between 25-35°C
                bmu.temps = [25.0 + 5.0 * (t % 60) / 60 + random.gauss(0, 1),
                             27.0 + 5.0 * (t % 60) / 60 + random.gauss(0, 1)]
                bmu.balancing_cells = random.randint(0, 0x3FF) if random.random() < 0.05 else 0
                bmu.need_balance = bmu.balancing_cells > 0

                # Occasional warning faults for demo
                if random.random() < 0.02:
                    bmu.ov_warning = 1 << random.randint(0, 9)
                else:
                    bmu.ov_warning = 0

                bmu.last_update = time.time()
                self.decoder.last_rx_time = time.time()

            time.sleep(0.5)

# =============================================================================
# Configuration Dialog
# =============================================================================
class ConfigDialog:
    def __init__(self, parent, on_apply: Callable):
        self.on_apply = on_apply
        self.win = tk.Toplevel(parent)
        self.win.title("Configuration")
        self.win.geometry("320x200")
        self.win.configure(bg=C['panel'])
        self.win.resizable(False, False)
        self.win.transient(parent)
        self.win.grab_set()

        # Center on parent
        self.win.update_idletasks()
        x = parent.winfo_x() + (parent.winfo_width() - 320) // 2
        y = parent.winfo_y() + (parent.winfo_height() - 200) // 2
        self.win.geometry(f"+{x}+{y}")

        self._create_widgets()

    def _create_widgets(self):
        # Title
        tk.Label(self.win, text="Runtime Configuration", font=('Consolas', 12, 'bold'),
                 fg=C['accent'], bg=C['panel']).pack(pady=(15, 10))

        form = tk.Frame(self.win, bg=C['panel'])
        form.pack(fill='x', padx=20)

        # MODULE_NUM
        row1 = tk.Frame(form, bg=C['panel'])
        row1.pack(fill='x', pady=5)
        tk.Label(row1, text="Module Count:", font=('Consolas', 10), fg=C['text'],
                 bg=C['panel'], width=14, anchor='w').pack(side='left')
        self.module_var = tk.StringVar(value=str(Config.MODULE_NUM))
        tk.Spinbox(row1, from_=1, to=16, textvariable=self.module_var, width=8,
                   font=('Consolas', 10), bg=C['cell'], fg=C['text'],
                   buttonbackground=C['cell']).pack(side='left', padx=5)

        # CAN Bitrate
        row2 = tk.Frame(form, bg=C['panel'])
        row2.pack(fill='x', pady=5)
        tk.Label(row2, text="CAN Bitrate:", font=('Consolas', 10), fg=C['text'],
                 bg=C['panel'], width=14, anchor='w').pack(side='left')
        self.bitrate_var = tk.StringVar(value=str(Config.CAN_BITRATE))
        combo = ttk.Combobox(row2, textvariable=self.bitrate_var, width=10,
                             values=['125000', '250000', '500000', '1000000'])
        combo.pack(side='left', padx=5)

        # Note
        tk.Label(self.win, text="Note: Changes require reconnection",
                 font=('Consolas', 8), fg=C['dim'], bg=C['panel']).pack(pady=10)

        # Buttons
        btn_frame = tk.Frame(self.win, bg=C['panel'])
        btn_frame.pack(pady=10)
        tk.Button(btn_frame, text="Apply", font=('Consolas', 10, 'bold'),
                  fg=C['bg'], bg=C['accent'], bd=0, width=10,
                  command=self._apply).pack(side='left', padx=5)
        tk.Button(btn_frame, text="Cancel", font=('Consolas', 10),
                  fg=C['text'], bg=C['cell'], bd=0, width=10,
                  command=self.win.destroy).pack(side='left', padx=5)

    def _apply(self):
        try:
            new_modules = int(self.module_var.get())
            new_bitrate = int(self.bitrate_var.get())

            if new_modules < 1 or new_modules > 16:
                raise ValueError("Module count must be 1-16")
            if new_bitrate not in [125000, 250000, 500000, 1000000]:
                raise ValueError("Invalid bitrate")

            self.on_apply(new_modules, new_bitrate)
            self.win.destroy()
        except ValueError as e:
            messagebox.showerror("Invalid Input", str(e))

# =============================================================================
# Main GUI
# =============================================================================
class BMSMonitor:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.decoder = BMSDecoder()
        self.can_src: Optional[CANSource] = None
        self.demo: Optional[DemoGenerator] = None
        self.view = 'overview'  # 'overview' or module_id (int)

        self._setup_window()
        self._create_header()
        self._create_main()
        self._create_status_bar()
        self._update_loop()

    def _setup_window(self):
        self.root.title("BP16B BMS")
        self.root.geometry("1680x1020")  # ~20% larger
        self.root.configure(bg=C['bg'])
        self.root.minsize(1400, 850)
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(1, weight=1)

    def _create_header(self):
        hdr = tk.Frame(self.root, bg=C['header'], height=75)  # 60->75
        hdr.grid(row=0, column=0, sticky='ew', padx=12, pady=(12, 6))
        hdr.grid_propagate(False)

        # Title
        tk.Label(hdr, text="BP16B BMS", font=('Consolas', 20, 'bold'),  # 16->20
                 fg=C['accent'], bg=C['header']).pack(side='left', padx=24)

        # Config button
        tk.Button(hdr, text="\u2699", font=('Consolas', 18), fg=C['dim'],  # 14->18
                  bg=C['header'], bd=0, cursor='hand2',
                  command=self._show_config).pack(side='left', padx=6)

        # AMS_OK indicator (center) - shows fault reason
        self.ams_frame = tk.Frame(hdr, bg=C['ok'], highlightbackground=C['border'],
                                  highlightthickness=2)
        self.ams_frame.pack(side='left', expand=True)
        self.ams_lbl = tk.Label(self.ams_frame, text="AMS", font=('Consolas', 11),  # 9->11
                                fg=C['bg'], bg=C['ok'])
        self.ams_lbl.pack(padx=14, pady=(4, 0))
        self.ams_status = tk.Label(self.ams_frame, text="OK", font=('Consolas', 15, 'bold'),  # 12->15
                                   fg=C['bg'], bg=C['ok'])
        self.ams_status.pack(padx=14, pady=(0, 4))

        # Controls
        ctrl = tk.Frame(hdr, bg=C['header'])
        ctrl.pack(side='right', padx=24)

        tk.Label(ctrl, text="IF:", font=('Consolas', 11), fg=C['dim'],  # 9->11
                 bg=C['header']).pack(side='left')
        self.if_var = tk.StringVar(value='slcan')
        ttk.Combobox(ctrl, textvariable=self.if_var,
                     values=['slcan', 'pcan', 'socketcan', 'kvaser'],
                     width=10, state='readonly', font=('Consolas', 10)).pack(side='left', padx=3)

        tk.Label(ctrl, text="CH:", font=('Consolas', 11), fg=C['dim'],  # 9->11
                 bg=C['header']).pack(side='left', padx=(6, 0))
        self.ch_var = tk.StringVar(value='/dev/ttyACM0')
        tk.Entry(ctrl, textvariable=self.ch_var, font=('Consolas', 11), width=16,  # 9->11
                 bg=C['cell'], fg=C['text'], insertbackground=C['text']).pack(side='left', padx=3)

        self.conn_btn = tk.Button(ctrl, text="CONNECT", font=('Consolas', 11, 'bold'),  # 9->11
                                  fg=C['bg'], bg=C['accent'], bd=0, width=12,
                                  command=self._toggle_connect)
        self.conn_btn.pack(side='left', padx=6)

        self.demo_btn = tk.Button(ctrl, text="DEMO", font=('Consolas', 11),  # 9->11
                                  fg=C['text'], bg=C['cell'], bd=0, width=8,
                                  command=self._toggle_demo)
        self.demo_btn.pack(side='left', padx=3)

    def _create_main(self):
        self.main = tk.Frame(self.root, bg=C['bg'])
        self.main.grid(row=1, column=0, sticky='nsew', padx=10, pady=5)
        self.main.grid_columnconfigure(0, weight=3)
        self.main.grid_columnconfigure(1, weight=1)
        self.main.grid_rowconfigure(0, weight=1)

        self._create_overview()
        self._create_detail()
        self._create_summary()
        self._show_overview()

    def _create_overview(self):
        self.overview = tk.Frame(self.main, bg=C['bg'])
        self.cards = {}
        self._rebuild_overview()

    def _rebuild_overview(self):
        """Rebuild overview grid when module count changes"""
        for widget in self.overview.winfo_children():
            widget.destroy()
        self.cards.clear()

        # Configure grid
        cols = 2
        rows = (Config.MODULE_NUM + 1) // 2
        for c in range(cols):
            self.overview.grid_columnconfigure(c, weight=1)
        for r in range(rows):
            self.overview.grid_rowconfigure(r, weight=1)

        # Create cards
        for idx, mid in enumerate(range(1, Config.MODULE_NUM + 1)):
            card = self._create_card(self.overview, mid)
            card.grid(row=idx // 2, column=idx % 2, sticky='nsew', padx=4, pady=4)
            self.cards[mid] = card

    def _create_card(self, parent, mid):
        card = tk.Frame(parent, bg=C['panel'], highlightbackground=C['border'],
                        highlightthickness=2, cursor='hand2')  # 1->2
        card.bind('<Button-1>', lambda e, m=mid: self._show_detail(m))
        card._mid = mid
        card._cells = []
        card._temps = []

        # Header
        hdr = tk.Frame(card, bg=C['panel'])
        hdr.pack(fill='x', padx=10, pady=(8, 4))
        card._title = tk.Label(hdr, text=f"BMU {mid}", font=('Consolas', 14, 'bold'),  # 11->14
                               fg=C['accent'], bg=C['panel'])
        card._title.pack(side='left')
        card._volt = tk.Label(hdr, text="--V", font=('Consolas', 12),  # 10->12
                              fg=C['text'], bg=C['panel'])
        card._volt.pack(side='right')

        # Cells 2x5
        cells_frame = tk.Frame(card, bg=C['panel'])
        cells_frame.pack(fill='x', padx=8, pady=3)
        for i in range(Config.CELL_NUM):
            cf = tk.Frame(cells_frame, bg=C['cell'], width=65, height=44)  # 52x36 -> 65x44
            cf.grid(row=i // 5, column=i % 5, padx=2, pady=2)
            cf.grid_propagate(False)
            tk.Label(cf, text=f"C{i + 1}", font=('Consolas', 7), fg=C['dim'],  # 6->7
                     bg=C['cell']).pack(anchor='nw', padx=3)
            lbl = tk.Label(cf, text="-.--", font=('Consolas', 11, 'bold'),  # 9->11
                           fg=C['text'], bg=C['cell'])
            lbl.pack(expand=True)
            card._cells.append(lbl)

        # Info row
        info = tk.Frame(card, bg=C['panel'])
        info.pack(fill='x', padx=10, pady=(3, 8))
        for t in range(2):
            tk.Label(info, text=f"T{t + 1}:", font=('Consolas', 10), fg=C['dim'],  # 8->10
                     bg=C['panel']).pack(side='left')
            tl = tk.Label(info, text="--C", font=('Consolas', 10), fg=C['text'],  # 8->10
                          bg=C['panel'])
            tl.pack(side='left', padx=(0, 10))
            card._temps.append(tl)
        tk.Label(info, text="dV:", font=('Consolas', 10), fg=C['dim'],  # 8->10
                 bg=C['panel']).pack(side='left')
        card._dv = tk.Label(info, text="--mV", font=('Consolas', 10),  # 8->10
                            fg=C['text'], bg=C['panel'])
        card._dv.pack(side='left')
        card._bal = tk.Label(info, text="", font=('Consolas', 10),  # 8->10
                             fg=C['bal'], bg=C['panel'])
        card._bal.pack(side='right')

        # Bind clicks on all children
        for w in card.winfo_children():
            w.bind('<Button-1>', lambda e, m=mid: self._show_detail(m))
            for c in w.winfo_children():
                c.bind('<Button-1>', lambda e, m=mid: self._show_detail(m))

        return card

    def _create_detail(self):
        self.detail = tk.Frame(self.main, bg=C['bg'])

        # Header
        dhdr = tk.Frame(self.detail, bg=C['header'], height=55)  # 45->55
        dhdr.pack(fill='x', pady=(0, 6))
        dhdr.pack_propagate(False)
        tk.Button(dhdr, text="< Back", font=('Consolas', 11), fg=C['text'],  # 9->11
                  bg=C['cell'], bd=0, command=self._show_overview).pack(side='left', padx=12, pady=10)
        self.det_title = tk.Label(dhdr, text="BMU --", font=('Consolas', 15, 'bold'),  # 12->15
                                  fg=C['accent'], bg=C['header'])
        self.det_title.pack(side='left', padx=18)
        self.det_conn = tk.Label(dhdr, text="", font=('Consolas', 11),  # 9->11
                                 fg=C['ok'], bg=C['header'])
        self.det_conn.pack(side='right', padx=18)

        content = tk.Frame(self.detail, bg=C['bg'])
        content.pack(fill='both', expand=True)

        # Left: Cells + Status
        left = tk.Frame(content, bg=C['panel'])
        left.pack(side='left', fill='both', expand=True, padx=(0, 6))

        tk.Label(left, text="Cell Voltages", font=('Consolas', 12, 'bold'),  # 10->12
                 fg=C['accent'], bg=C['panel']).pack(anchor='w', padx=12, pady=(12, 6))
        cg = tk.Frame(left, bg=C['panel'])
        cg.pack(fill='x', padx=12)
        self.det_cells = []
        for i in range(Config.CELL_NUM):
            cf = tk.Frame(cg, bg=C['cell'], width=80, height=55)  # 65x45 -> 80x55
            cf.grid(row=i // 5, column=i % 5, padx=3, pady=3)
            cf.grid_propagate(False)
            tk.Label(cf, text=f"Cell {i + 1}", font=('Consolas', 8), fg=C['dim'],  # 7->8
                     bg=C['cell']).pack(anchor='nw', padx=4)
            lbl = tk.Label(cf, text="-.---V", font=('Consolas', 12, 'bold'),  # 10->12
                           fg=C['text'], bg=C['cell'])
            lbl.pack(expand=True)
            self.det_cells.append(lbl)

        tk.Label(left, text="Status", font=('Consolas', 12, 'bold'),  # 10->12
                 fg=C['accent'], bg=C['panel']).pack(anchor='w', padx=12, pady=(18, 6))
        sf = tk.Frame(left, bg=C['panel'])
        sf.pack(fill='x', padx=12)
        self.det_temps = []
        for t in range(2):
            tk.Label(sf, text=f"Temp {t + 1}:", font=('Consolas', 11), fg=C['dim'],  # 9->11
                     bg=C['panel']).pack(side='left')
            tl = tk.Label(sf, text="--C", font=('Consolas', 11, 'bold'),  # 9->11
                          fg=C['text'], bg=C['panel'])
            tl.pack(side='left', padx=(4, 18))
            self.det_temps.append(tl)
        tk.Label(sf, text="dV:", font=('Consolas', 11), fg=C['dim'],  # 9->11
                 bg=C['panel']).pack(side='left')
        self.det_dv = tk.Label(sf, text="--mV", font=('Consolas', 11, 'bold'),  # 9->11
                               fg=C['text'], bg=C['panel'])
        self.det_dv.pack(side='left', padx=4)
        self.det_bal = tk.Label(sf, text="", font=('Consolas', 11),  # 9->11
                                fg=C['bal'], bg=C['panel'])
        self.det_bal.pack(side='right', padx=12)

        # Right: Fault table
        right = tk.Frame(content, bg=C['panel'])
        right.pack(side='right', fill='both', expand=True, padx=(6, 0))

        tk.Label(right, text="Fault Status", font=('Consolas', 12, 'bold'),  # 10->12
                 fg=C['accent'], bg=C['panel']).pack(anchor='w', padx=12, pady=(12, 6))
        tbl = tk.Frame(right, bg=C['panel'])
        tbl.pack(fill='x', padx=12)

        # Header row
        hr = tk.Frame(tbl, bg=C['cell'])
        hr.pack(fill='x', pady=(0, 3))
        tk.Label(hr, text="Fault", font=('Consolas', 10), fg=C['dim'],  # 8->10
                 bg=C['cell'], width=8, anchor='w').pack(side='left', padx=3)
        for i in range(Config.CELL_NUM):
            tk.Label(hr, text=f"C{i + 1}", font=('Consolas', 10), fg=C['dim'],  # 8->10
                     bg=C['cell'], width=4).pack(side='left', padx=1)

        self.fault_cells = {}
        for attr, name, is_crit in FAULT_TYPES:
            row = tk.Frame(tbl, bg=C['panel'])
            row.pack(fill='x', pady=2)
            tk.Label(row, text=name, font=('Consolas', 10), fg=C['text'],  # 8->10
                     bg=C['cell'], width=8, anchor='w').pack(side='left', padx=3)
            for ci in range(Config.CELL_NUM):
                lbl = tk.Label(row, text="\u2014", font=('Consolas', 11),  # 9->11
                               fg=C['dim'], bg=C['cell'], width=4)
                lbl.pack(side='left', padx=1)
                self.fault_cells[(attr, ci)] = lbl

        # Legend
        leg = tk.Frame(right, bg=C['panel'])
        leg.pack(anchor='w', padx=12, pady=12)
        for sym, col, txt in [("\u2713", C['ok'], "OK"), ("\u26A0", C['warn'], "Warn"),
                              ("\u2717", C['crit'], "Crit")]:
            tk.Label(leg, text=f"{sym} {txt}", font=('Consolas', 10),  # 8->10
                     fg=col, bg=C['panel']).pack(side='left', padx=10)

    def _create_summary(self):
        self.summary = tk.Frame(self.main, bg=C['panel'])
        tk.Label(self.summary, text="PACK SUMMARY", font=('Consolas', 13, 'bold'),  # 11->13
                 fg=C['accent'], bg=C['panel']).pack(pady=(15, 10))

        sf = tk.Frame(self.summary, bg=C['panel'])
        sf.pack(fill='x', padx=15)
        self.sum_lbl = {}
        for key, name, dflt in [('total', 'Total Voltage', '--- V'),
                                ('min', 'Min Cell', '-.-- V'),
                                ('max', 'Max Cell', '-.-- V'),
                                ('dv', 'Pack dV', '--- mV'),
                                ('temp', 'Max Temp', '--C'),
                                ('conn', 'Connected', '0/8'),
                                ('fault', 'Faults', '0')]:
            r = tk.Frame(sf, bg=C['panel'])
            r.pack(fill='x', pady=3)
            tk.Label(r, text=name, font=('Consolas', 11), fg=C['dim'],  # 9->11
                     bg=C['panel']).pack(side='left')
            l = tk.Label(r, text=dflt, font=('Consolas', 12, 'bold'),  # 10->12
                         fg=C['text'], bg=C['panel'])
            l.pack(side='right')
            self.sum_lbl[key] = l

        tk.Frame(self.summary, bg=C['border'], height=1).pack(fill='x', padx=15, pady=15)

        # Module status grid
        tk.Label(self.summary, text="MODULE STATUS", font=('Consolas', 11),  # 9->11
                 fg=C['dim'], bg=C['panel']).pack()
        self.mod_grid_frame = tk.Frame(self.summary, bg=C['panel'])
        self.mod_grid_frame.pack(pady=10)
        self.mod_ind = {}
        self._rebuild_module_grid()

    def _rebuild_module_grid(self):
        """Rebuild module status grid when count changes"""
        for widget in self.mod_grid_frame.winfo_children():
            widget.destroy()
        self.mod_ind.clear()

        for i in range(Config.MODULE_NUM):
            l = tk.Label(self.mod_grid_frame, text=f"\u25CBM{i + 1}",
                         font=('Consolas', 10), fg=C['dim'], bg=C['cell'], width=6)  # 8->10, width 5->6
            l.grid(row=i // 4, column=i % 4, padx=3, pady=3)
            self.mod_ind[i + 1] = l

    def _show_overview(self):
        self.detail.grid_forget()
        self.overview.grid(row=0, column=0, sticky='nsew')
        self.summary.grid(row=0, column=1, sticky='nsew')
        self.view = 'overview'

    def _show_detail(self, mid):
        if mid not in self.decoder.modules:
            return
        self.overview.grid_forget()
        self.summary.grid_forget()
        self.detail.grid(row=0, column=0, columnspan=2, sticky='nsew')
        self.det_title.configure(text=f"BMU {mid}")
        self.view = mid

    def _show_config(self):
        ConfigDialog(self.root, self._apply_config)

    def _apply_config(self, module_count: int, bitrate: int):
        """Apply configuration changes"""
        # Disconnect if connected
        if self.can_src and self.can_src.running:
            self._toggle_connect()
        if self.demo and self.demo.running:
            self._toggle_demo()

        # Update config
        Config.MODULE_NUM = module_count
        Config.CAN_BITRATE = bitrate

        # Rebuild decoder and UI
        self.decoder.set_module_count(module_count)
        self._rebuild_overview()
        self._rebuild_module_grid()
        self._show_overview()

        self.status_lbl.configure(
            text=f"Config: {module_count} modules, {bitrate // 1000}k bitrate",
            fg=C['accent'])

    def _create_status_bar(self):
        sb = tk.Frame(self.root, bg=C['panel'], height=35)  # 28->35
        sb.grid(row=2, column=0, sticky='ew', padx=12, pady=(6, 12))
        sb.grid_propagate(False)
        self.status_lbl = tk.Label(sb, text="Disconnected", font=('Consolas', 11),  # 9->11
                                   fg=C['dim'], bg=C['panel'])
        self.status_lbl.pack(side='left', padx=12, pady=5)
        self.rx_lbl = tk.Label(sb, text="RX: 0", font=('Consolas', 11),  # 9->11
                               fg=C['dim'], bg=C['panel'])
        self.rx_lbl.pack(side='right', padx=12, pady=5)

    def _toggle_connect(self):
        if self.can_src and self.can_src.running:
            self.can_src.disconnect()
            self.can_src = None
            self.decoder.reset()
            self.conn_btn.configure(text="CONNECT", bg=C['accent'], fg=C['bg'])
            self.status_lbl.configure(text="Disconnected", fg=C['dim'])
        else:
            if self.demo and self.demo.running:
                self._toggle_demo()
            self.can_src = CANSource(self.decoder)
            if self.can_src.connect(self.if_var.get(), self.ch_var.get(), Config.CAN_BITRATE):
                self.conn_btn.configure(text="DISCONNECT", bg=C['crit'], fg=C['text'])
                self.status_lbl.configure(
                    text=f"CAN: {self.if_var.get()}:{self.ch_var.get()} @ {Config.CAN_BITRATE // 1000}k",
                    fg=C['accent'])
            else:
                messagebox.showerror("Error", f"CAN connection failed: {self.can_src.error_msg}")
                self.can_src = None

    def _toggle_demo(self):
        if self.demo and self.demo.running:
            self.demo.stop()
            self.demo = None
            self.decoder.reset()
            self.demo_btn.configure(text="DEMO", bg=C['cell'], fg=C['text'])
            self.status_lbl.configure(text="Disconnected", fg=C['dim'])
        else:
            if self.can_src and self.can_src.running:
                self._toggle_connect()
            self.demo = DemoGenerator(self.decoder)
            self.demo.start()
            self.demo_btn.configure(text="STOP", bg=C['warn'], fg=C['bg'])
            self.status_lbl.configure(text="Demo Mode", fg=C['warn'])

    def _update_loop(self):
        # Aggregate and evaluate AMS status
        self.decoder.aggregate()
        self._update_ams_ok()
        self._update_rx()

        if self.view == 'overview':
            self._update_overview()
            self._update_summary()
        else:
            self._update_detail_view(self.view)

        self.root.after(100, self._update_loop)

    def _update_ams_ok(self):
        """Update AMS_OK indicator with fault reason"""
        ok = self.decoder.ams_ok
        reason = self.decoder.fault_reason

        col = C['ok'] if ok else C['crit']
        txt = reason.value

        self.ams_frame.configure(bg=col)
        self.ams_lbl.configure(bg=col)
        self.ams_status.configure(text=txt, bg=col)

    def _update_rx(self):
        rx = self.can_src.rx_count if self.can_src else 0
        self.rx_lbl.configure(text=f"RX: {rx}")

    def _update_overview(self):
        for mid, card in self.cards.items():
            if mid not in self.decoder.modules:
                continue
            bmu = self.decoder.modules[mid]
            conn = bmu.is_connected()

            if conn:
                card.configure(bg=C['panel'])
                card._title.configure(fg=C['accent'], bg=C['panel'])
                card._volt.configure(text=f"{bmu.total_v():.1f}V", fg=C['text'], bg=C['panel'])

                for i, v in enumerate(bmu.cell_voltages):
                    if i < len(card._cells):
                        fg = (C['crit'] if v >= Config.CELL_V_HIGH else
                              C['warn'] if v <= Config.CELL_V_LOW else C['ok'])
                        card._cells[i].configure(text=f"{v:.2f}", fg=fg, bg=C['cell'])

                for t, temp in enumerate(bmu.temps):
                    if t < len(card._temps):
                        card._temps[t].configure(text=f"{temp:.0f}C")

                card._dv.configure(text=f"{bmu.delta_v * 1000:.0f}mV")
                card._bal.configure(text="\u26A1BAL" if bmu.balancing_cells else "")

                border = (C['crit'] if bmu.has_critical() else
                          C['warn'] if bmu.has_fault() else C['accent'])
                card.configure(highlightbackground=border)
            else:
                card.configure(bg=C['disconn'], highlightbackground=C['disconn'])
                card._title.configure(fg=C['disconn'], bg=C['disconn'])
                card._volt.configure(text="OFFLINE", fg=C['disconn'], bg=C['disconn'])
                for cl in card._cells:
                    cl.configure(text="--", fg=C['disconn'], bg=C['disconn'])

    def _update_summary(self):
        d = self.decoder

        self.sum_lbl['total'].configure(
            text=f"{d.accum_voltage:.1f} V" if d.accum_voltage > 0 else "--- V")
        self.sum_lbl['min'].configure(
            text=f"{d.min_cell_v:.2f} V" if d.min_cell_v > 0 else "-.-- V")
        self.sum_lbl['max'].configure(
            text=f"{d.max_cell_v:.2f} V" if d.max_cell_v > 0 else "-.-- V")
        self.sum_lbl['dv'].configure(
            text=f"{d.pack_delta_v * 1000:.0f} mV" if d.pack_delta_v > 0 else "--- mV")
        self.sum_lbl['temp'].configure(
            text=f"{d.max_temp:.0f}C" if d.max_temp > 0 else "--C")
        self.sum_lbl['conn'].configure(text=f"{d.connected_count}/{Config.MODULE_NUM}")
        self.sum_lbl['fault'].configure(
            text=str(d.fault_count),
            fg=C['crit'] if d.fault_count > 0 else C['ok'])

        # Update module indicators
        for mid, bmu in self.decoder.modules.items():
            if mid not in self.mod_ind:
                continue
            if bmu.is_connected():
                sym = "\u2717" if bmu.has_critical() else "\u26A0" if bmu.has_fault() else "\u25CF"
                col = C['crit'] if bmu.has_critical() else C['warn'] if bmu.has_fault() else C['ok']
            else:
                sym, col = "\u25CB", C['disconn']
            self.mod_ind[mid].configure(text=f"{sym}M{mid}", fg=col)

    def _update_detail_view(self, mid):
        if mid not in self.decoder.modules:
            return

        bmu = self.decoder.modules[mid]
        conn = bmu.is_connected()

        self.det_conn.configure(
            text="\u25CF Connected" if conn else "\u25CB Offline",
            fg=C['ok'] if conn else C['disconn'])

        if conn:
            for i, v in enumerate(bmu.cell_voltages):
                if i < len(self.det_cells):
                    fg = (C['crit'] if v >= Config.CELL_V_HIGH else
                          C['warn'] if v <= Config.CELL_V_LOW else C['ok'])
                    self.det_cells[i].configure(text=f"{v:.3f}V", fg=fg)

            for t, temp in enumerate(bmu.temps):
                if t < len(self.det_temps):
                    self.det_temps[t].configure(text=f"{temp:.1f}C")

            self.det_dv.configure(text=f"{bmu.delta_v * 1000:.0f}mV")

            bal_list = [str(i + 1) for i in range(Config.CELL_NUM)
                        if bmu.balancing_cells & (1 << (9 - i))]
            self.det_bal.configure(text=f"\u26A1 {','.join(bal_list)}" if bal_list else "")

            for attr, name, is_crit in FAULT_TYPES:
                for ci in range(Config.CELL_NUM):
                    key = (attr, ci)
                    if key not in self.fault_cells:
                        continue
                    has = bmu.cell_fault(attr, ci)
                    sym = "\u2717" if has and is_crit else "\u26A0" if has else "\u2713"
                    col = C['crit'] if has and is_crit else C['warn'] if has else C['ok']
                    self.fault_cells[key].configure(text=sym, fg=col)
        else:
            for cl in self.det_cells:
                cl.configure(text="-.---V", fg=C['disconn'])
            for tl in self.det_temps:
                tl.configure(text="--C", fg=C['disconn'])
            self.det_dv.configure(text="--mV")
            self.det_bal.configure(text="")
            for k in self.fault_cells:
                self.fault_cells[k].configure(text="\u2014", fg=C['dim'])

    def on_closing(self):
        if self.can_src:
            self.can_src.disconnect()
        if self.demo:
            self.demo.stop()
        self.root.destroy()

# =============================================================================
# Main
# =============================================================================
def main():
    parser = argparse.ArgumentParser(description='BP16B BMS CAN Monitor')
    parser.add_argument('--demo', '-d', action='store_true', help='Start in demo mode')
    parser.add_argument('--can', '-c', type=str,
                        help='CAN interface: slcan, pcan, socketcan, kvaser')
    parser.add_argument('--channel', type=str,
                        help='CAN channel: /dev/ttyACM0, PCAN_USBBUS1, can0')
    parser.add_argument('--bitrate', type=int, default=Config.CAN_BITRATE,
                        help=f'CAN bitrate (default: {Config.CAN_BITRATE})')
    parser.add_argument('--modules', type=int, default=Config.MODULE_NUM,
                        help=f'Number of BMU modules (default: {Config.MODULE_NUM})')
    args = parser.parse_args()

    # Apply command line config
    if args.bitrate:
        Config.CAN_BITRATE = args.bitrate
    if args.modules:
        Config.MODULE_NUM = args.modules

    root = tk.Tk()
    app = BMSMonitor(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)

    if args.demo:
        root.after(500, app._toggle_demo)
    elif args.can and args.channel:
        app.if_var.set(args.can)
        app.ch_var.set(args.channel)
        root.after(500, app._toggle_connect)

    root.mainloop()

if __name__ == '__main__':
    main()
