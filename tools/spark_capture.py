#!/usr/bin/env python3
"""
spark_capture.py — Read Spark presets from the ESP32 capture device.

The ESP32 (tools/spark_capture_esp32/) simulates a Spark 40 amp over
Bluetooth Classic. When the Spark Android app sends a preset, the ESP32
prints a JSON: line over USB serial. This script reads that line and saves
it as a PedalinoMini-compatible .json file.

Requirements:
    pip install pyserial

Usage:
    python spark_capture.py [COM_PORT] [output_dir]

    COM_PORT   defaults to auto-detect (first ESP32-like port)
    output_dir defaults to ./presets/
"""

import json
import os
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial not found — run: pip install pyserial")

OUTPUT_DIR = sys.argv[2] if len(sys.argv) > 2 else "presets"


def find_port() -> str:
    if len(sys.argv) > 1:
        return sys.argv[1]
    # Auto-detect: prefer CP210x / CH340 (common ESP32 USB-serial chips)
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        if any(k in desc for k in ("cp210", "ch340", "ch341", "uart", "esp")):
            print(f"Auto-detected port: {p.device} ({p.description})")
            return p.device
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if len(ports) == 1:
        print(f"Using only available port: {ports[0]}")
        return ports[0]
    if ports:
        print(f"Available ports: {', '.join(ports)}")
        print("Pass the port as first argument: python spark_capture.py COM3")
        sys.exit(1)
    sys.exit("No serial ports found. Is the ESP32 plugged in?")


def save_preset(preset: dict) -> str:
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    raw  = preset.get("Name", "preset")
    safe = "".join(c if c.isalnum() or c in " -_" else "_" for c in raw)
    safe = safe.strip().replace(" ", "-")
    base = os.path.join(OUTPUT_DIR, safe[:27])
    path = base + ".json"
    n = 1
    while os.path.exists(path):
        path = f"{base}_{n}.json"; n += 1
    with open(path, "w") as f:
        json.dump(preset, f, indent=2)
    return path


def main():
    port = find_port()
    print(f"Opening {port} at 115200 baud...")
    ser = serial.Serial(port, 115200, timeout=1)
    time.sleep(2)   # let ESP32 boot / reset settle
    ser.reset_input_buffer()

    print(f"Output: {os.path.abspath(OUTPUT_DIR)}/")
    print("In the Spark app: My Tones -> tap a preset -> Send to Amp")
    print("Ctrl+C to stop.\n")

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if not line:
                continue
            if line.startswith("STATUS:"):
                print(" ", line[7:])
            elif line.startswith("JSON:"):
                raw = line[5:]
                try:
                    preset = json.loads(raw)
                    path = save_preset(preset)
                    print(f"  Saved -> {path}")
                except json.JSONDecodeError as e:
                    print(f"  [!] JSON parse error: {e}")
                    print(f"  Raw: {raw[:120]}...")
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print("\nStopped.")


if __name__ == "__main__":
    main()
