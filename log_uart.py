import datetime
import tkinter as tk

def log_uart(msg, uart_log_queue):
    timestamp = datetime.datetime.now().strftime("%H:%M:%S")
    full_msg = f"[{timestamp}] {msg}"
    print(full_msg)
    if uart_log_queue is not None:
        uart_log_queue.put(full_msg)

