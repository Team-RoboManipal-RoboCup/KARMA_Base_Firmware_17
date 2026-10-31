#!/usr/bin/env python3
"""
laptop_robot_test.py
====================
Interactive Robot Test & Telemetry Dashboard for Autonomous 4-Wheel Bot.
Communicates with STM32F411 over USB Virtual COM Port (CDC).

Features:
- Live JSON telemetry parsing: 3-Encoder Dead Wheel Odometry + BNO085 (x, y, z, yaw)
- Real-time command transmission (individual motor speeds, array formats, broadcast PWM)
- Interactive WASD keyboard drive mode
- Direct JSON command prompt
- Compatible with Laptop (Windows/macOS/Linux) and Nvidia Jetson
"""

import sys
import time
import json
import threading
import serial
import serial.tools.list_ports

try:
    import msvcrt  # Windows non-blocking keyboard input
    WINDOWS_CONSOLE = True
except ImportError:
    import select
    import termios
    import tty
    WINDOWS_CONSOLE = False


class RobotBridge:
    def __init__(self, port=None, baudrate=115200):
        self.port_name = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False
        self.thread = None

        # Telemetry state
        self.last_telemetry = {}
        self.telemetry_lock = threading.Lock()
        self.packet_count = 0
        self.last_packet_time = 0

    def find_stm32_port(self):
        ports = serial.tools.list_ports.comports()
        stm_ports = []
        for p in ports:
            desc = p.description.lower()
            hwid = p.hwid.lower()
            if "stm" in desc or "virtual" in desc or "0483" in hwid or "stlink" in desc:
                stm_ports.append(p.device)

        if stm_ports:
            return stm_ports[0]
        elif ports:
            return ports[0].device
        return None

    def connect(self):
        if not self.port_name:
            self.port_name = self.find_stm32_port()
            if not self.port_name:
                print("\n[ERROR] No COM ports found! Please connect the STM32 via USB.")
                return False

        try:
            print(f"[INFO] Connecting to STM32 on {self.port_name} at {self.baudrate} baud...")
            self.ser = serial.Serial(self.port_name, self.baudrate, timeout=0.1)
            self.running = True
            self.thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.thread.start()
            print(f"[SUCCESS] Connected to {self.port_name}!\n")
            return True
        except Exception as e:
            print(f"[ERROR] Failed to open {self.port_name}: {e}")
            return False

    def disconnect(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.send_stop()
            self.ser.close()
        print("[INFO] Disconnected.")

    def _rx_loop(self):
        while self.running and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                if line.startswith("{") and line.endswith("}"):
                    try:
                        data = json.loads(line)
                        with self.telemetry_lock:
                            self.last_telemetry = data
                            self.packet_count += 1
                            self.last_packet_time = time.time()
                    except json.JSONDecodeError:
                        pass
            except Exception:
                break

    def send_raw(self, cmd_str):
        if self.ser and self.ser.is_open:
            try:
                payload = (cmd_str.strip() + "\r\n").encode('utf-8')
                self.ser.write(payload)
            except Exception as e:
                print(f"[ERROR] Send failed: {e}")

    def send_motor_speeds(self, m1, m2, m3, m4):
        """Send individual speeds (-1000 to +1000) for all 4 motors"""
        cmd = json.dumps({"m": [int(m1), int(m2), int(m3), int(m4)]})
        self.send_raw(cmd)

    def send_broadcast_pwm(self, pwm_val):
        """Send common PWM (-1000 to +1000) to all 4 motors"""
        cmd = json.dumps({"pwm": int(pwm_val)})
        self.send_raw(cmd)

    def send_stop(self):
        """Emergency stop all motors"""
        cmd = json.dumps({"cmd": "stop"})
        self.send_raw(cmd)

    def get_telemetry(self):
        with self.telemetry_lock:
            return dict(self.last_telemetry)


def get_key_nonblocking():
    """Reads a single keypress without waiting for Enter."""
    if WINDOWS_CONSOLE:
        if msvcrt.kbhit():
            ch = msvcrt.getch()
            if ch in [b'\x00', b'\xe0']:  # Special keys/arrows
                ch = msvcrt.getch()
            try:
                return ch.decode('utf-8').lower()
            except UnicodeDecodeError:
                return None
        return None
    else:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            if rlist:
                ch = sys.stdin.read(1)
                return ch.lower()
            return None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def print_banner():
    print("=" * 65)
    print("      AUTONOMOUS 4-WHEEL BOT - LAPTOP / JETSON TEST SUITE      ")
    print("=" * 65)
    print(" 3-Encoder Dead Wheel Odometry + BNO085 IMU + 4x MD30C Motors ")
    print("=" * 65)


def run_dashboard(bridge):
    print("\n--- [MODE 1: LIVE TELEMETRY STREAM & KEYBOARD DRIVE] ---")
    print(" Controls:")
    print("   [W] Forward        [S] Backward       [A] Turn Left      [D] Turn Right")
    print("   [SPACE] Stop All   [+] Increase Speed [-] Decrease Speed [Q] Back to Menu")
    print("-" * 65)

    drive_speed = 400
    last_print = 0

    while True:
        key = get_key_nonblocking()
        if key:
            if key == 'q':
                bridge.send_stop()
                break
            elif key == 'w':
                bridge.send_motor_speeds(drive_speed, drive_speed, drive_speed, drive_speed)
            elif key == 's':
                bridge.send_motor_speeds(-drive_speed, -drive_speed, -drive_speed, -drive_speed)
            elif key == 'a':
                bridge.send_motor_speeds(-drive_speed, drive_speed, -drive_speed, drive_speed)
            elif key == 'd':
                bridge.send_motor_speeds(drive_speed, -drive_speed, drive_speed, -drive_speed)
            elif key == ' ':
                bridge.send_stop()
            elif key in ['+', '=']:
                drive_speed = min(1000, drive_speed + 50)
                print(f"\n[INFO] Drive speed set to: {drive_speed}")
            elif key in ['-', '_']:
                drive_speed = max(50, drive_speed - 50)
                print(f"\n[INFO] Drive speed set to: {drive_speed}")

        now = time.time()
        if now - last_print >= 0.1:  # 10 Hz display refresh
            last_print = now
            telem = bridge.get_telemetry()
            if telem:
                e1 = telem.get("e1", telem.get("enc", {}).get("e1", 0))
                e2 = telem.get("e2", telem.get("enc", {}).get("e2", 0))
                e3 = telem.get("e3", telem.get("enc", {}).get("e3", 0))

                bx = telem.get("x", telem.get("bno", {}).get("x", 0.0))
                by = telem.get("y", telem.get("bno", {}).get("y", 0.0))
                bz = telem.get("z", telem.get("bno", {}).get("z", 0.0))
                yaw = telem.get("yaw", telem.get("bno", {}).get("yaw", 0.0))
                bno_ok = telem.get("bno_ok", 0)

                bno_str = f"YAW:{yaw:6.2f}° | ACC:[X:{bx:5.2f} Y:{by:5.2f} Z:{bz:5.2f}]" if bno_ok else "BNO:[INITIALIZING/WAITING DATA]"

                sys.stdout.write(
                    f"\rENC:[E1:{e1:6d}|E2:{e2:6d}|E3:{e3:6d}] | {bno_str} | SPD:{drive_speed:4d}  "
                )
                sys.stdout.flush()

        time.sleep(0.02)


def run_command_prompt(bridge):
    print("\n--- [MODE 2: DIRECT JSON / SERIAL COMMAND PROMPT] ---")
    print(" Examples:")
    print("   {\"m\": [500, 500, -500, -500]}              (Set 4 motor speeds)")
    print("   {\"m1\": 300, \"m2\": 300, \"m3\": 0, \"m4\": 0}    (Individual motors)")
    print("   {\"pwm\": 400}                               (Broadcast all motors)")
    print("   {\"cmd\": \"stop\"}                            (Emergency stop)")
    print("   50                                         (Plain 50% duty shortcut)")
    print(" Type 'exit' to return to menu.")
    print("-" * 65)

    while True:
        try:
            cmd = input("\nRobot Command > ").strip()
            if not cmd:
                continue
            if cmd.lower() in ['exit', 'quit', 'q']:
                bridge.send_stop()
                break
            bridge.send_raw(cmd)
            print(f"[SENT] -> {cmd}")
            time.sleep(0.1)
            telem = bridge.get_telemetry()
            print(f"[TELEMETRY] <- {json.dumps(telem)}")
        except (KeyboardInterrupt, EOFError):
            bridge.send_stop()
            break


def main():
    print_banner()

    # Discover ports
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("[ERROR] No serial ports found. Please connect the STM32 via USB and retry.")
        return

    print("Available Serial Ports:")
    for idx, p in enumerate(ports):
        print(f" [{idx + 1}] {p.device}: {p.description}")

    selected_port = None
    if len(ports) == 1:
        selected_port = ports[0].device
        print(f"[INFO] Auto-selected: {selected_port}")
    else:
        choice = input(f"Select port (1-{len(ports)}) or press Enter for auto: ").strip()
        if choice.isdigit() and 1 <= int(choice) <= len(ports):
            selected_port = ports[int(choice) - 1].device

    bridge = RobotBridge(port=selected_port)
    if not bridge.connect():
        return

    try:
        while True:
            print("\nSelect Option:")
            print(" [1] Live Telemetry Dashboard & Keyboard WASD Drive")
            print(" [2] Direct JSON / Serial Command Prompt")
            print(" [3] Motor Spin Test (M1 -> M2 -> M3 -> M4 test)")
            print(" [4] Exit")

            opt = input("Choice (1-4): ").strip()
            if opt == '1':
                run_dashboard(bridge)
            elif opt == '2':
                run_command_prompt(bridge)
            elif opt == '3':
                print("\n[TEST] Running bidirectional individual motor test sequence...")
                for m_idx in range(4):
                    print(f" -> Spinning Motor {m_idx + 1} FORWARD (+300) for 1.2s...")
                    speeds = [0, 0, 0, 0]
                    speeds[m_idx] = 300
                    bridge.send_motor_speeds(*speeds)
                    time.sleep(1.2)
                    bridge.send_stop()
                    time.sleep(0.4)
                    print(f" -> Spinning Motor {m_idx + 1} REVERSE (-300) for 1.2s...")
                    speeds = [0, 0, 0, 0]
                    speeds[m_idx] = -300
                    bridge.send_motor_speeds(*speeds)
                    time.sleep(1.2)
                    bridge.send_stop()
                    time.sleep(0.4)
                print("[TEST] Bidirectional motor sequence complete!")
            elif opt == '4':
                break
    finally:
        bridge.disconnect()
        print("\n[INFO] Exited test suite.")


if __name__ == "__main__":
    main()
