import tkinter as tk
from array import array
from itertools import count
from sys import maxsize
from tkinter import filedialog, messagebox
from tkinter import ttk
from turtledemo.penrose import start

from PIL import Image, ImageTk
import cv2
import threading
import time
import serial
from serial.tools import list_ports
import recv_phone_image
import global_var
from Image2Gcode import genGcode
from RobotGcodeGen import RobotGcodeGen
from serial import threaded
import queue
import datetime
import matplotlib.pyplot as plt
import os
import warnings
from EvaluateResult import consine_similarity as validate, extract_frame
from log_uart import log_uart

ENCODING = 'utf-8'
IMG_WIDTH = 640
IMG_HEIGHT = 480
MAX_LOG_LINES = 500
PACKAGE_SIZE = 64 # 24

error_codes_to_message = [
    (1, "Expected command letter", "Expected command letter", "G-code words consist of a letter and a value. Letter was not found."),
    (2, "Bad number format", "Bad number format", "Missing the expected G-code word value or numeric value format is not valid."),
    (3, "Invalid statement", "Invalid statement", "Grbl '$' system command was not recognized or supported."),
    (4, "Value < 0", "Value < 0", "Negative value received for an expected positive value."),
    (5, "Setting disabled", "Setting disabled", "Homing cycle failure. Homing is not enabled via settings."),
    (6, "Value < 3 usec", "Value < 3 usec", "Minimum step pulse time must be greater than 3usec."),
    (7, "EEPROM read fail", "EEPROM read fail. Using defaults", "An EEPROM read failed. Auto-restoring affected EEPROM to default values."),
    (8, "Not idle", "Not idle", "Grbl '$' command cannot be used unless Grbl is IDLE. Ensures smooth operation during a job."),
    (9, "G-code lock", "G-code lock", "G-code commands are locked out during alarm or jog state."),
    (10, "Homing not enabled", "Homing not enabled", "Soft limits cannot be enabled without homing also enabled."),
    (11, "Line overflow", "Line overflow", "Max characters per line exceeded. Received command line was not executed."),
    (12, "Step rate > 30kHz", "Step rate > 30kHz", "Grbl '$' setting value cause the step rate to exceed the maximum supported."),
    (13, "Check Door", "Check Door", "Safety door detected as opened and door state initiated."),
    (14, "Line length exceeded", "Line length exceeded", "Build info or startup line exceeded EEPROM line length limit. Line not stored."),
    (15, "Travel exceeded", "Travel exceeded", "Jog target exceeds machine travel. Jog command has been ignored."),
    (16, "Invalid jog command", "Invalid jog command", "Jog command has no '=' or contains prohibited g-code."),
    (17, "Laser mode requires PWM", "Setting disabled", "Laser mode requires PWM output."),
    (20, "Unsupported command", "Unsupported command", "Unsupported or invalid g-code command found in block."),
    (21, "Modal group violation", "Modal group violation", "More than one g-code command from same modal group found in block."),
    (22, "Undefined feed rate", "Undefined feed rate", "Feed rate has not yet been set or is undefined."),
    (23, "Invalid gcode ID:23", "Invalid gcode ID:23", "G-code command in block requires an integer value."),
    (24, "Invalid gcode ID:24", "Invalid gcode ID:24", "More than one g-code command that requires axis words found in block."),
    (25, "Invalid gcode ID:25", "Invalid gcode ID:25", "Repeated g-code word found in block."),
    (26, "Invalid gcode ID:26", "Invalid gcode ID:26", "No axis words found in block for g-code command or current modal state which requires them."),
    (27, "Invalid gcode ID:27", "Invalid gcode ID:27", "Line number value is invalid."),
    (28, "Invalid gcode ID:28", "Invalid gcode ID:28", "G-code command is missing a required value word."),
    (29, "Invalid gcode ID:29", "Invalid gcode ID:29", "G59.x work coordinate systems are not supported."),
    (30, "Invalid gcode ID:30", "Invalid gcode ID:30", "G53 only allowed with G0 and G1 motion modes."),
    (31, "Invalid gcode ID:31", "Invalid gcode ID:31", "Axis words found in block when no command or current modal state uses them."),
    (32, "Invalid gcode ID:32", "Invalid gcode ID:32", "G2 and G3 arcs require at least one in-plane axis word."),
    (33, "Invalid gcode ID:33", "Invalid gcode ID:33", "Motion command target is invalid."),
    (34, "Invalid gcode ID:34", "Invalid gcode ID:34", "Arc radius value is invalid."),
    (35, "Invalid gcode ID:35", "Invalid gcode ID:35", "G2 and G3 arcs require at least one in-plane offset word."),
    (36, "Invalid gcode ID:36", "Invalid gcode ID:36", "Unused value words found in block."),
    (37, "Invalid gcode ID:37", "Invalid gcode ID:37", "G43.1 dynamic tool length offset is not assigned to configured tool length axis."),
    (38, "Invalid gcode ID:38", "Invalid gcode ID:38", "Tool number greater than max supported value.")
]

alarm_codes_to_message = [
    (1, "Hard limit", "Hard limit", "Hard limit has been triggered. Machine position is likely lost due to sudden halt. Re-homing is highly recommended."),
    (2, "Soft limit", "Soft limit", "Soft limit alarm. G-code motion target exceeds machine travel. Machine position retained. Alarm may be safely unlocked."),
    (3, "Abort during cycle", "Abort during cycle", "Reset while in motion. Machine position is likely lost due to sudden halt. Re-homing is highly recommended."),
    (4, "Probe fail", "Probe fail", "Probe is not in the expected initial state before starting probe cycle."),
    (5, "Probe fail", "Probe fail", "Probe did not contact the workpiece within the programmed travel."),
    (6, "Homing fail", "Homing fail", "The active homing cycle was reset."),
    (7, "Homing fail", "Homing fail", "Safety door was opened during homing cycle."),
    (8, "Homing fail", "Homing fail", "Pull off travel failed to clear limit switch. Try increasing pull-off."),
    (9, "Homing fail", "Homing fail", "Could not find limit switch. Try increasing max travel or check wiring.")
]

grbl_settings = [
    ('$0', 'Step pulse time', 'µs', 'Sets time length per step. Minimum 3µs.'),
    ('$1', 'Step idle delay', 'ms', 'Hold delay before disabling steppers. 255 keeps motors enabled.'),
    ('$2', 'Step pulse invert', 'mask', 'Inverts the step signal. Set axis bit to invert (00000ZYX).'),
    ('$3', 'Step direction invert', 'mask', 'Inverts the direction signal. Set axis bit to invert (00000ZYX).'),
    ('$4', 'Invert step enable pin', 'bool', 'Inverts the stepper driver enable pin signal.'),
    ('$5', 'Invert limit pins', 'bool', 'Inverts all limit input pins.'),
    ('$6', 'Invert probe pin', 'bool', 'Inverts the probe input pin signal.'),
    ('$10', 'Status report options', 'mask', 'Alters data included in status reports.'),
    ('$11', 'Junction deviation', 'mm', 'Controls speed through junctions. Lower = slower.'),
    ('$12', 'Arc tolerance', 'mm', 'Sets G2/G3 arc tracing accuracy.'),
    ('$13', 'Report in inches', 'bool', 'Enables inch units in reports.'),
    ('$20', 'Soft limits enable', 'bool', 'Enable soft limits. Requires homing.'),
    ('$21', 'Hard limits enable', 'bool', 'Enable hard limits. Triggers alarm on switch.'),
    ('$22', 'Homing cycle enable', 'bool', 'Enable homing cycle. Requires limit switches.'),
    ('$23', 'Homing direction invert', 'mask', 'Invert homing direction. Bitmask (00000ZYX).'),
    ('$24', 'Homing locate feed rate', 'mm/min', 'Slow rate to locate switch accurately.'),
    ('$25', 'Homing search seek rate', 'mm/min', 'Fast rate to find limit switch.'),
    ('$26', 'Homing debounce delay', 'ms', 'Delay to debounce switch during homing.'),
    ('$27', 'Homing pull-off distance', 'mm', 'Retract after switch trigger. Must clear switch.'),
    ('$30', 'Max spindle speed', 'RPM', 'Spindle speed at 100% PWM duty.'),
    ('$31', 'Min spindle speed', 'RPM', 'Spindle speed at 0.4% PWM duty.'),
    ('$32', 'Laser-mode enable', 'bool', 'Enable laser mode. Avoids halts on spindle changes.'),
    ('$100', 'X steps/mm', 'steps/mm', 'Steps per mm for X-axis.'),
    ('$101', 'Y steps/mm', 'steps/mm', 'Steps per mm for Y-axis.'),
    ('$102', 'Z steps/mm', 'steps/mm', 'Steps per mm for Z-axis.'),
    ('$110', 'X max rate', 'mm/min', 'Maximum movement rate for X-axis.'),
    ('$111', 'Y max rate', 'mm/min', 'Maximum movement rate for Y-axis.'),
    ('$112', 'Z max rate', 'mm/min', 'Maximum movement rate for Z-axis.'),
    ('$120', 'X acceleration', 'mm/sec^2', 'Acceleration for X-axis. Avoid step loss.'),
    ('$121', 'Y acceleration', 'mm/sec^2', 'Acceleration for Y-axis. Avoid step loss.'),
    ('$122', 'Z acceleration', 'mm/sec^2', 'Acceleration for Z-axis. Avoid step loss.'),
    ('$130', 'X max travel', 'mm', 'Max travel distance for X from home.'),
    ('$131', 'Y max travel', 'mm', 'Max travel distance for Y from home.'),
    ('$132', 'Z max travel', 'mm', 'Max travel distance for Z from home.'),
]


def show_error_codes_window(root):
    win = tk.Toplevel(root)
    win.title("📘 GRBL Error Codes")
    win.geometry("800x500")

    tree = ttk.Treeview(win, columns=("type", "code", "short_msg", "old_msg", "description"), show="headings")
    tree.pack(fill="both", expand=True)

    # Đặt tên cột
    tree.heading("type", text="Type")
    tree.heading("code", text="Code")
    tree.heading("short_msg", text="Short Message")
    tree.heading("old_msg", text="Old Message")
    tree.heading("description", text="Description")

    # Cài đặt chiều rộng các cột
    tree.column("type", width=80, anchor="center")
    tree.column("code", width=50, anchor="center")
    tree.column("short_msg", width=180)
    tree.column("old_msg", width=180)
    tree.column("description", width=400)

    # Thêm dữ liệu ERROR
    for code, short_msg, old_msg, desc in error_codes_to_message:
        tree.insert("", "end", values=("ERROR", code, short_msg, old_msg, desc))


def show_alarm_codes_window(root):
    win = tk.Toplevel(root)
    win.title("📘 GRBL Alarm Codes")
    win.geometry("800x500")

    tree = ttk.Treeview(win, columns=("type", "code", "short_msg", "old_msg", "description"), show="headings")
    tree.pack(fill="both", expand=True)

    # Đặt tên cột
    tree.heading("type", text="Type")
    tree.heading("code", text="Code")
    tree.heading("short_msg", text="Short Message")
    tree.heading("old_msg", text="Old Message")
    tree.heading("description", text="Description")

    # Cài đặt chiều rộng các cột
    tree.column("type", width=80, anchor="center")
    tree.column("code", width=50, anchor="center")
    tree.column("short_msg", width=180)
    tree.column("old_msg", width=180)
    tree.column("description", width=400)
    for code, short_msg, old_msg, desc in alarm_codes_to_message:
        tree.insert("", "end", values=("ALARM", code, short_msg, old_msg, desc))

def show_setting_codes_window(root):
    win = tk.Toplevel(root)
    win.title("⚙️ GRBL Setting Codes")
    win.geometry("800x500")
    tree = ttk.Treeview(win, columns=("code", "name", "unit", "desc"), show="headings")
    tree.pack(fill="both", expand=True)
    tree.heading("code", text="$-Code")
    tree.heading("name", text="Setting")
    tree.heading("unit", text="Unit")
    tree.heading("desc", text="Description")
    tree.column("code", width=60)
    tree.column("name", width=200)
    tree.column("unit", width=80)
    tree.column("desc", width=440)
    for row in grbl_settings:
        tree.insert("", "end", values=row)

# def log_uart(msg):
#     timestamp = datetime.datetime.now().strftime("%H:%M:%S")
#     full_msg = f"[{timestamp}] {msg}"
#     print(full_msg)
#     if hasattr(App, 'uart_log_box') and App.uart_log_box:
#         App.uart_log_box.configure(state='normal')
#         App.uart_log_box.insert(tk.END, full_msg + "\n")
#         App.uart_log_box.see(tk.END)
#
#         # if int(App.uart_log_box.index('end-1c').split('.')[0]) > MAX_LOG_LINES:
#         #     App.uart_log_box.delete('1.0', '2.0')
#
#         App.uart_log_box.configure(state='disabled')

class SerialCommunication(serial.threaded.LineReader):
    TERMINATOR = b'\r\n'
    def __init__(self,uart_log_queue=None):
        super().__init__()
        self.ok_received = False
        self.done_count = 0
        self.ok_count = 0
        self.done_lock = threading.Lock()
        self.error = False
        self.uart_log_queue = uart_log_queue
        self.stop_done = False

    def handle_line(self, line):
        log_uart(f"⬅️ RX: {line.strip()}", self.uart_log_queue)
        if line.strip() == "ok":
            self.ok_received = True
        elif line.strip() == "done":
            with self.done_lock:
                self.done_count += 1
        elif line.startswith("error"):
            self.error = True

    def wait_for_stop_done(self, timeout=2):
        self.stop_done = False
        start = time.time()
        while time.time() - start < timeout:
            if self.stop_done:
                return True
        return False

    def wait_for_receive_ok(self, timeout=0.5):
        start = time.time()
        self.ok_received = False
        self.error = False
        while time.time() - start < timeout:
            if self.ok_received:
                return True
            elif self.error:
                return False
            self.ok_received = False
            self.error = False
        # time.sleep(0.5)
        return False

    def get_ok_count(self, max_count=PACKAGE_SIZE, timeout=1.0):
        return self.ok_count;

    def get_done_count(self, max_count=PACKAGE_SIZE, timeout=1.0):
        with self.done_lock:
            done = self.done_count
            self.done_count = 0
        return done

    def reset_count_queue(self):
        if self.ok_received and self.done_count:
            self.ok_received = False
            self.done_count = 0

def find_uart_port():
    ports = list_ports.comports()
    for port, desc, hwid in ports:
        if "ttyACM" in port or "USB" in port or "ACM" in port or "COM" in port:
            return port
    return None

def send_uart_command(protocol, cmd, wait_ok=True, retries=3):
    for _ in range(retries):
        protocol.ok_received = False
        msg = f"{cmd}\n"
        protocol.transport.write(msg.encode(ENCODING))
        if hasattr(protocol, 'uart_log_queue') and protocol.uart_log_queue is not None:
            log_uart(f"➡️ TX: {cmd}", protocol.uart_log_queue)
        if wait_ok:
            ok = protocol.wait_for_receive_ok()
            if ok: return ok
        else:
            time.sleep(0.5)
            break
    return False


def reset_system(protocol):
    protocol.transport.write(b'\x18')
    send_uart_command(protocol, "$X")
    send_uart_command(protocol, "M5")
    send_uart_command(protocol, "G28")
    send_uart_command(protocol, "F2000")
    send_uart_command(protocol, "G1 X0 Y0 Z0 A0 B0")
    send_uart_command(protocol, "G92 X0 Y0 Z0 A0 B0")
    protocol.get_done_count()
    protocol.reset_count_queue()

def run_initialization_sequence(protocol, stop_event):
    cmds = ["$X", "$HX", "$HY", "$HZ", "$HA", "$HB", "G92 X0 Y0 Z0 A0 B0"]
    for cmd in cmds:
        if stop_event.is_set(): return
        send_uart_command(protocol, cmd, True)
    protocol.reset_count_queue()

def read_gcode_file(filename):
    gcode_lines = []
    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if '(' in line:
                line = line.split('(')[0].strip()
            if line:
                gcode_lines.append(line)
    return gcode_lines

def is_blocking_command(cmd):
    return cmd.startswith("M") or cmd.startswith("G92") or cmd.startswith("G28")

def is_motion_command(cmd):
    return cmd.startswith("G0") or cmd.startswith("G1")

def send_gcode_package(app_instance, protocol, gcode_lines, total_cmds, package, shared_state, stop_event, shared_state_lock):
    size = min(PACKAGE_SIZE - shared_state['on_flight'], package)
    for _ in range(size):
        if stop_event.is_set() or shared_state['sent'] >= total_cmds:
            return False
        app_instance.stop_event.clear()
        cmd = gcode_lines[shared_state['sent']]
        shared_state['sent'] += 1
        if is_motion_command(cmd):
            send_uart_command(protocol, cmd, True)
            shared_state['on_flight'] += is_motion_command(cmd)
        else:
            send_uart_command(protocol, cmd, False)
            shared_state['received'] += is_blocking_command(cmd)
        print(
            f"Đã gửi: {shared_state['sent']}, on flight: {shared_state['on_flight']}")
    return True

def receive_gcode_done(protocol, receive_signal, temp_signal, stop_event, done_queue):
    done_count = 0
    done_idx = 0
    start_time = time.time()
    while not stop_event.is_set():
        num = protocol.get_done_count()
        if num > 0:
            done_count += num
        end_time = time.time()

        if end_time - start_time > 0.05 and done_count > 0 and not done_queue.full():
            print(f"done = {done_count}")
            done_queue.put(done_count)
            done_count = 0
            done_idx += 1
            start_time = end_time
            # if done_idx == 3: first_time = True

        if done_idx == 1:
            temp_signal.set()
        else:
            if not receive_signal.wait(timeout=0.05): continue
            receive_signal.clear()
            temp_signal.set()

    while not done_queue.empty():
        done = done_queue.get()
        print(f"done = {done}")

def temp_thread(app_instance, protocol, total_cmds, done_arr, temp_arr, shared_state, send_signal, temp_signal, stop_event, arr_lock, shared_state_lock, done_queue):
    while not stop_event.is_set():
        app_instance.stop_event.clear()
        if not temp_signal.wait(timeout=0.1): continue
        if not done_queue.empty():
            done = done_queue.get()
            print(f"done pop: {done}")
            with shared_state_lock:
                shared_state['on_flight'] -= done
                shared_state['received'] += done
                print(f"Giảm on_flight: {shared_state['on_flight']}")
                if shared_state['received'] >= total_cmds and shared_state['on_flight'] < 1:
                    print(f"Complete - received: {shared_state['received']}, on flight{shared_state['on_flight']}")
                    stop_event.set()  # Dừng tất cả luồng
                    break
        temp_signal.clear()
        send_signal.set()


def send_gcode_file(app_instance, protocol, gcode_lines, total_cmds, shared_state, send_signal, receive_signal, stop_event, done_queue, shared_state_lock):
    sent_done = 0
    queue_empty = 0
    protocol.reset_count_queue()
    with shared_state_lock:
        send_gcode_package(app_instance, protocol, gcode_lines, total_cmds, 18, shared_state, stop_event, shared_state_lock)

    while not stop_event.is_set():
        if not send_signal.wait(timeout=0.05): continue
        app_instance.stop_event.clear()
        if not done_queue.empty():
            done = done_queue.get()
            shared_state['on_flight'] -= done
            shared_state['received'] += done
            print(f"Receive: {shared_state['received']}, on flight: {shared_state['on_flight']} ")
        else:
            if queue_empty > 0: continue
            print("Queue empty")
            queue_empty += 1

        if shared_state['received'] >= total_cmds and shared_state['on_flight'] < 1:
            print(f"Complete - received: {shared_state['received']}, on flight{shared_state['on_flight']}")
            stop_event.set()  # Dừng tất cả luồng
            break
        else:
            success = send_gcode_package(app_instance, protocol, gcode_lines, total_cmds, PACKAGE_SIZE, shared_state, stop_event,
                                     shared_state_lock)

        if not success:
            if sent_done > 1: continue
            print("Sent done")
            sent_done += 1

        send_signal.clear()
        receive_signal.set()



def get_serial_ports():
    ports = list_ports.comports()
    print("DEBUG | Các cổng phát hiện:", [port.device for port in ports])
    return [port.device for port in ports]
class App:
    def __init__(self, root):
        self.port_var = tk.StringVar()
        self.serial_port_dropdown = None  # OptionMenu sẽ lưu ở đây
        self.serial_ports = []  # Danh sách COM port

        self.gcode_lines = None
        self.uart_log_box = None
        self.manual_entry = None
        self.gcode_entry = None
        self.left_img_label = None
        self.right_img_label = None
        self.source_var = None
        self.root = root
        self.root.title("📟 G-code Control Panel")
        self.protocol = None
        self.cap = None
        self.running = True
        self.last_frame = None
        self.show_mirror = True
        self.done_arr = [0] * 3000
        self.temp_arr = [0] * 3000
        self.done_queue = queue.Queue(maxsize=PACKAGE_SIZE)
        self.shared_state_lock = threading.Lock()
        self.arr_lock = threading.Lock()
        self.is_simulate_image = False
        # self.command_queue = queue.Queue(maxsize=36)
        self.queue_lock = threading.Lock()
        self.send_signal = threading.Event()
        self.temp_signal = threading.Event()
        self.receive_signal = threading.Event()
        self.stop_event = threading.Event()
        self.shared_state = {
        'sent': 0,
        'received': 0,
        'on_flight': 0
        # 'done_all_signal': False
        }
        self.gcode_file_path = None
        self.gcode_path_var = tk.StringVar(value="Chưa chọn file G-code")
        self.start_recv_thread()
        self.ok_received = False
        self.uart_log_queue = queue.Queue()
        self.done_queue = queue.Queue()
        self.ok_queue = queue.Queue()

        self.build_gui()
        self.start_serial()
        self.process_uart_log_queue()
        self.start_camera(0)

    def build_gui(self):
        img_frame = ttk.Frame(self.root)
        img_frame.pack(pady=10)

        self.left_img_label = tk.Label(img_frame)
        self.left_img_label.pack(side="left", padx=10)

        self.right_img_label = tk.Label(img_frame)
        self.right_img_label.pack(side="left", padx=10)

        source_frame = ttk.Frame(self.root)
        source_frame.pack(pady=5)
        self.source_var = tk.StringVar(value="in_camera")
        ttk.Label(source_frame, text="Source:").pack(side="left")
        ttk.Radiobutton(source_frame, text="Camera Laptop", variable=self.source_var, value="in_camera", command=self.switch_source).pack(side="left")
        ttk.Radiobutton(source_frame, text="Camera ngoài", variable=self.source_var, value="ex_camera", command=self.switch_source).pack(side="left")
        ttk.Button(source_frame, text="📂 Chọn ảnh", command=self.choose_image).pack(side="left", padx=5)
        ttk.Button(source_frame, text="📸 Chụp hình", command=self.capture_frame).pack(side="left", padx=5)
        ttk.Button(source_frame, text="Xử lý ảnh", command=self.image_processing).pack(side="left", padx=5)
        ttk.Button(source_frame, text="Sinh gcode", command=self.generate_gcode).pack(side="left", padx=5)

        button_frame = ttk.Frame(self.root)
        button_frame.pack(pady=10)

        gcode_frame = ttk.Frame(button_frame)
        gcode_frame.pack(side="left", padx=5)
        self.gcode_entry = ttk.Entry(gcode_frame, width=40, textvariable=self.gcode_path_var, state="readonly")
        self.gcode_entry.pack(side="left")
        ttk.Button(gcode_frame, text="📂", width=3, command=self.choose_gcode_file).pack(side="left", padx=2)

        ttk.Button(button_frame, text="📤 Gửi gcode", width=12, command=self.do_send_gcode).pack(side="left", padx=5)
        ttk.Button(button_frame, text="🛑 Dừng", width=12, command=self.do_stop).pack(side="left", padx=5)
        ttk.Button(button_frame, text="🔁 Tiếp tục", width=12, command=self.do_continue_gcode).pack(side="left", padx=5)
        ttk.Button(button_frame, text="🏠 Homing", width=12, command=lambda: threading.Thread(target=self.do_homing, daemon=True).start()).pack(side="left", padx=5)
        ttk.Button(button_frame, text="🔄 Reset", width=8, command=lambda: threading.Thread(target=self.do_reset, daemon=True).start()).pack(side="left", padx=5)
        info_frame = ttk.Frame(self.root)
        info_frame.pack(pady=5)
        code_button_frame = ttk.Frame(self.root)
        code_button_frame.pack(anchor="c", padx=10, pady=5)

        ttk.Button(code_button_frame, text="📘 Error Codes", width=14,
                   command=lambda: show_error_codes_window(self.root)).pack(side="left", padx=5)
        ttk.Button(code_button_frame, text="⚙️ Setting Codes", width=16,
                   command=lambda: show_setting_codes_window(self.root)).pack(side="left", padx=5)
        ttk.Button(code_button_frame, text="🚨 Alarm Codes", width=14,
                   command=lambda: show_alarm_codes_window(self.root)).pack(side="left", padx=5)
        ttk.Button(button_frame, text="Đánh giá", width=8,
                   command=lambda: threading.Thread(target=self.validate_result_window, daemon=True).start()).pack(side="left", padx=5)
        manual_frame = ttk.Frame(self.root)
        manual_frame.pack(pady=5)
        self.manual_entry = ttk.Entry(manual_frame, width=40)
        self.manual_entry.pack(side="left", padx=5)
        self.manual_entry.bind("<Return>", lambda event: self.send_manual_command())
        ttk.Button(manual_frame, text="📨 Gửi lệnh", command=self.send_manual_command).pack(side="left", padx=5)

        log_frame = ttk.LabelFrame(self.root, text="📜 UART Log Terminal")
        log_frame.pack(fill="both", expand=True, padx=10, pady=10)
        self.uart_log_box = tk.Text(log_frame, height=15, wrap="word", bg="white", fg="navy", insertbackground="white")
        self.uart_log_box.pack(fill="both", expand=True)
        self.uart_log_box.configure(state='disabled')

        port_frame = ttk.Frame(self.root)
        port_frame.pack(pady=1000)
        ttk.Label(port_frame, text="Cổng UART:").pack(side="right")

        self.serial_ports = get_serial_ports()
        if not self.serial_ports:
            self.serial_ports = ["None"]
        self.port_var.set(self.serial_ports[0])

        self.serial_port_dropdown = tk.OptionMenu(
            port_frame, self.port_var, *self.serial_ports
        )
        self.serial_port_dropdown.pack(side="left", padx=5)

        ttk.Button(port_frame, text="Kết nối lại", command=self.restart_serial).pack(side="left", padx=5)
        self.process_uart_log_queue()

    def image_processing(self):
        if global_var.index_capture_image:
            def thread_job():
                genGcode.main()
                self.root.after(0, lambda: messagebox.showinfo("Thành công", "Xử lý ảnh thành công"))
            threading.Thread(target=thread_job, daemon=True).start()

    def generate_gcode(self):
        if global_var.index_capture_image:
            def thread_job():
                RobotGcodeGen.main()
                self.show_mirror = True
                global_var.is_finish_covert_image = False
                self.root.after(0, lambda: messagebox.showinfo("Thành công", "Sinh gcode thành công"))
            threading.Thread(target=thread_job, daemon=True).start()

    def choose_gcode_file(self):
        filepath = filedialog.askopenfilename(
            initialdir="final_gcode/face_gcode",
            title="Chọn File gcode",
            filetypes=[("Gcode files", "*.gcode *.nc *.txt")]
        )
        if filepath:
            self.gcode_file_path = filepath
            self.gcode_path_var.set(filepath)

    def do_send_gcode(self):
        if not self.gcode_file_path:
            messagebox.showerror("Lỗi", "Bạn chưa chọn file G-code.")
            return
        self.shared_state = {
            'sent': 0,
            'received': 0,
            'on_flight': 0
            # 'done_all_signal': False
        }
        threading.Thread(target=self.send_gcode_in_background, args=(self.uart_log_queue,), daemon=True).start()

    def process_uart_log_queue(self):
        try:
            while True:
                msg = self.uart_log_queue.get_nowait()
                self.uart_log_box.configure(state='normal')
                self.uart_log_box.insert(tk.END, msg + "\n")
                # Tối đa 300 dòng log, xóa dòng đầu nếu quá
                if int(float(self.uart_log_box.index('end-1c').split('.')[0])) > 300:
                    self.uart_log_box.delete('1.0', '2.0')
                self.uart_log_box.see(tk.END)
                self.uart_log_box.configure(state='disabled')
        except queue.Empty:
            pass
        self.root.after(50, self.process_uart_log_queue)

    def do_continue_gcode(self):
        if self.gcode_file_path:
            self.stop_event.clear()
            threading.Thread(target=self.send_gcode_in_background, daemon=True).start()

    def do_stop(self):
        return

    def validate_result_window(self):
        if global_var.index_capture_image:
            input_path = f"input_capture_excam/{global_var.index_capture_image}.jpg"
            image_name = f"{global_var.index_capture_image}.jpg"
            extract_frame.preprocessing(image_name)
            original_image_path = f"Image2Gcode/output_image/{global_var.index_capture_image}_binary.jpg"
            image_path = f"EvaluateResult/output_image/{global_var.index_capture_image}.jpg"
            feature_similarity = validate.calculate_feature_similarity(original_image_path, image_path)
            print(f"Feature-based similarity (Cosine similarity) with {image_path}: {feature_similarity}")
            # Đọc ảnh để hiển thị
            img_original = cv2.imread(original_image_path)
            img_result = cv2.imread(image_path)

            # Chuyển BGR -> RGB để hiển thị đúng màu với matplotlib
            img_original = cv2.cvtColor(img_original, cv2.COLOR_BGR2RGB)
            img_result = cv2.cvtColor(img_result, cv2.COLOR_BGR2RGB)

            # Hiển thị ảnh và similarity
            plt.figure(figsize=(10, 4))

            plt.subplot(1, 2, 1)
            plt.imshow(img_original)
            plt.title("Original Binary Image")
            plt.axis("off")

            plt.subplot(1, 2, 2)
            plt.imshow(img_result)
            plt.title(f"Result Image\nSimilarity: {feature_similarity:.4f}")
            plt.axis("off")

            plt.tight_layout()
            plt.show()

    def run_gcode(self, gcode_lines):
        total_cmds = len(gcode_lines)
        self.stop_event.clear()
        self.shared_state = {'on_flight': 0, 'sent': 0, 'received': 0, 'blocking': 0}

        sender_thread = threading.Thread(
            target=send_gcode_file,
            args=(self, self.protocol, gcode_lines, total_cmds,
                  self.shared_state, self.send_signal, self.receive_signal, self.stop_event,
                  self.done_queue,self.shared_state_lock),
            daemon=True
        )

        receiver_thread = threading.Thread(target=receive_gcode_done,
                         args=(self.protocol, self.receive_signal,
                               self.send_signal, self.stop_event, self.done_queue),
                         daemon=True)

        sender_thread.start()
        receiver_thread.start()
        sender_thread.join()
        receiver_thread.join()

    def send_gcode_in_background(self, uart_box=None):
        def task():
            try:
                start_time = time.time()

                gcode_1 = read_gcode_file(self.gcode_file_path)
                self.run_gcode(gcode_1)

                gcode_2 = read_gcode_file("draw_rectangle.txt")
                self.run_gcode(gcode_2)

                end_time = time.time()
                log_uart(f"Execution time: {end_time - start_time:.2f}s", self.uart_log_queue)

            except Exception as e:
                log_uart(f"Lỗi khi gửi G-code: {str(e)}", self.uart_log_queue)

        thread = threading.Thread(target=task, daemon=True)
        thread.start()

    def send_manual_command(self):
        cmd = self.manual_entry.get().strip().upper()
        self.manual_entry.delete(0, tk.END)
        threading.Thread(target=self._manual_cmd_thread, args=(cmd,), daemon=True).start()

    def _manual_cmd_thread(self, cmd):
        if self.protocol:
            self.shared_state = {'on_flight': 0, 'sent': 0, 'received': 0}
            if cmd in ["CTRL X", "CTRL+X"]:
                self.protocol.transport.write(b'\x18')
            else:
                send_uart_command(self.protocol, cmd)

    def start_serial(self):
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Error", "Không tìm thấy cổng UART")
            return
        try:
            ser = serial.Serial(port, 921600, timeout=1)
            thread = serial.threaded.ReaderThread(ser, lambda: SerialCommunication(self.uart_log_queue))
            thread.start()
            self.protocol = thread.connect()[1]
            send_uart_command(self.protocol, "$$", wait_ok=False)
        except Exception as e:
            messagebox.showerror("Lỗi kết nối", str(e))

    def restart_serial(self):
        # Ngắt kết nối cũ nếu có
        if self.protocol and hasattr(self.protocol, 'transport'):
            try:
                self.protocol.transport.close()
            except:
                pass
        # Làm mới danh sách cổng
        ports = get_serial_ports()
        if not ports:
            ports = ["None"]
        menu = self.serial_port_dropdown["menu"]
        menu.delete(0, "end")
        for p in ports:
            menu.add_command(label=p, command=lambda v=p: self.port_var.set(v))
        self.port_var.set(ports[0])
        self.start_serial()

    def start_camera(self, camera_index=0):
        if self.cap:
            self.cap.release()
        self.cap = cv2.VideoCapture(camera_index)

        if self.cap.isOpened():
            # Cấu hình camera để giảm lag
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
        threading.Thread(target=self.capture_loop, daemon=True).start()

    def capture_loop(self):
        last_update = 0
        while self.running and self.cap and self.cap.isOpened():
            now = time.time()
            if now - last_update < 1 / 30:  # Giới hạn 30 FPS
                time.sleep(0.005)
                continue
            last_update = now
            if self.source_var == "ex_camera":
                self.cap.set(cv2.CAP_PROP_SATURATION, 0)
                self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 255)
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.flip(frame, 1)
                self.last_frame = frame
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                show_img = Image.fromarray(rgb).resize((IMG_WIDTH, IMG_HEIGHT))
                self.root.after(0, lambda img=show_img: self.display_image(self.left_img_label, img))
                if global_var.is_finish_covert_image:
                    image_handle_name = os.path.splitext(global_var.image_name)[0]
                    self.show_mirror = False
                    img_after_processing = (Image.open(f"Image2Gcode/output_image/{image_handle_name}_binary.jpg")
                                            .resize((IMG_WIDTH, IMG_HEIGHT)))
                    self.root.after(0, lambda img=img_after_processing: self.display_image(self.right_img_label, img))
                elif self.show_mirror:
                    self.root.after(0, lambda img=show_img: self.display_image(self.right_img_label, img))
                elif self.is_simulate_image:
                    image_handle_simulate_name = os.path.splitext(global_var.image_name)[0]
                    image_simulate = (Image.open(f"Image2Gcode/simulate_image/{image_handle_simulate_name}.png")
                                      .resize((IMG_WIDTH, IMG_HEIGHT)))
                    self.root.after(0, lambda img=image_simulate: self.display_image(self.right_img_label, img))
                elif global_var.is_get_image_from_phone:
                    phone_image = (Image.open(f"phone_image/{global_var.index_capture_image}.jpg")
                                   .resize((IMG_WIDTH, IMG_HEIGHT)))
                    self.root.after(0, lambda img=phone_image: self.display_image(self.right_img_label, img))
            time.sleep(0.005)

    def switch_source(self):
        cv2.setLogLevel(0)
        warnings.filterwarnings("ignore")
        # Force release camera nếu tồn tại
        if self.cap:
            self.cap.release()

        if self.source_var.get() == "ex_camera":
            self.start_camera(camera_index=1)
            self.show_mirror = True
        else:
            self.start_camera(camera_index=0)
            self.show_mirror = True

    def choose_image(self):
        global filename_image
        filepath = filedialog.askopenfilename(
            initialdir="Image2Gcode/input_image/capture",
            title="Chọn ảnh",
            filetypes=[("Image files", "*.jpg *.jpeg *.png *.bmp")]
        )
        if not filepath:
            return
        if filepath:
            filename_image = os.path.basename(filepath)
        global_var.image_name = ""
        global_var.image_name = filename_image
        global_var.is_finish_covert_image = False
        choose_img = Image.open(filepath).resize((IMG_WIDTH, IMG_HEIGHT))
        self.display_image(self.right_img_label, choose_img)
        messagebox.showinfo("Thành công", f"Chọn thành công ảnh {filename_image}.")
        global_var.is_choose_image = True
        global_var.is_get_image_from_phone = False
        global_var.is_capture = False
        self.is_simulate_image = False
        self.show_mirror = False

    def capture_frame(self):
        if self.last_frame is not None:
            global_var.is_finish_covert_image = False
            global_var.is_choose_image = False
            global_var.is_get_image_from_phone = False
            global_var.index_capture_image += 1
            global_var.is_capture = True
            global_var.image_name = ""
            global_var.image_name = f"{global_var.index_capture_image}.jpg"
            img = Image.fromarray(cv2.cvtColor(self.last_frame, cv2.COLOR_BGR2RGB)).resize((IMG_WIDTH, IMG_HEIGHT))
            if self.source_var.get() == "in_camera":
                img.save(f"Image2Gcode/input_image/capture/{global_var.index_capture_image}.jpg")
            else:
                img.save(f"EvaluateResult/input_capture_excam/{global_var.index_capture_image}.jpg")
            self.display_image(self.right_img_label, img)
            self.show_mirror = False
            self.is_simulate_image = False

    def start_recv_thread(self):
        def recv_loop():
            while True:
                try:
                    image_path = recv_phone_image.receive_file()
                    if image_path:
                        self.show_mirror = False
                        self.is_simulate_image = False
                        global_var.is_capture = False
                        global_var.is_choose_image = False
                        global_var.is_get_image_from_phone = True
                        global_var.is_finish_covert_image = False
                        img = Image.open(image_path)
                        img = img.resize((int(1836 * 480 / 3264), 480), Image.LANCZOS)
                        self.right_img_label.after(0, lambda: self.display_image(self.right_img_label, img))

                except Exception as e:
                    print(f"[!] Lỗi khi nhận ảnh: {e}")
                    time.sleep(1)  # Tránh chiếm CPU nếu có lỗi lặp

        # Tạo và khởi chạy thread
        recv_thread = threading.Thread(target=recv_loop, daemon=True)
        recv_thread.start()

    def reset_shared_state():
        return {'sent': 0, 'received': 0, 'on_flight': 0, 'blocking': 0}

    def display_image(self, label, img):
        imgtk = ImageTk.PhotoImage(img)
        label.imgtk = imgtk
        label.configure(image=imgtk)

    def do_homing(self):
        if self.protocol:
            run_initialization_sequence(self.protocol, self.stop_event)

    def do_reset(self):
        if self.protocol:
            reset_system(self.protocol)
        self.shared_state['on_flight'] = 0

    def on_close(self):
        self.running = False
        if self.cap:
            self.cap.release()
        self.root.destroy()

if __name__ == "__main__":
    # cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)
    os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'  # 0 = all logs, 1 = filter INFO, 2 = filter WARNING, 3 = filter ERROR
    os.environ['TF_ENABLE_ONEDNN_OPTS'] = '0'
    global_var.image_name = ""
    global_var.index_capture_image = 0
    global_var.is_capture = False
    global_var.is_finish_covert_image = False
    global_var.is_choose_image = False
    global_var.is_get_image_from_phone = False
    root = tk.Tk()
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
