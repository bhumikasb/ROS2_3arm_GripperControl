# --- wsg_interface.py ---
import socket
import time
import rclpy.logging
from wsg50_driver_pkg.wsg50_driver.constants import ERROR_CODES

class WSGInterface:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = None
        self.logger = rclpy.logging.get_logger('WSGInterface')

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(2.0)
        self.sock.connect((self.ip, self.port))
        self.logger.info(f"Connected to gripper at {self.ip}:{self.port}")
        self.send_cmd("VERBOSE=1")

    def reconnect(self):
        try:
            self.close()
            time.sleep(1)
            self.connect()
            self.logger.info("Reconnection successful")
            return True
        except Exception as e:
            self.logger.info(f"Reconnection failed: {e}")
            return False

    def close(self):
        if self.sock:
            try:
                self.send_cmd("BYE()")
                self.sock.close()
                self.logger.info("Connection closed")
            except Exception as e:
                self.logger.info(f"Error during close: {e}")
            self.sock = None

    def send(self, cmd):
        if not self.sock:
            raise ConnectionError("Not connected")
        self.logger.info(f"Sending command: {cmd}")
        self.sock.send((cmd + '\n').encode())

    def recv(self):
        data = self.sock.recv(4096).decode().strip()
        self.logger.info(f"Received response: {data}")
        return data

    def send_cmd(self, cmd):
        self.send(cmd)
        return self.recv()

    # Motion Commands
    def home(self, direction=None):
        """
        Homes (references) the gripper by executing the HOME() command.
        Optionally, pass direction: 0 (negative) or 1 (positive).
        Returns True if successful, False otherwise.
        """
        try:
            cmd = "HOME()" if direction is None else f"HOME({int(direction)})"
            self.logger.info(f"Homing gripper with command: {cmd}")
            self._ack_and_fin(cmd)
            self.logger.info("Homing complete.")
            return True
        except Exception as e:
            self.logger.error(f"Homing failed: {e}")
            return False
        

    def calibrate(self):
        """
        Calibrates the gripper by referencing (homing) in both directions.
        Determines the mechanical min and max positions.
        Returns True if successful, False otherwise.
        """
        try:
            self.logger.info("Starting calibration...")

            # Home in negative direction to find minimum mechanical position
            if not self.home(direction=0):
                raise RuntimeError("Homing in negative direction failed.")
            time.sleep(1.0)
            self.min_position = self.get_position()
            self.logger.info(f"Minimum mechanical position: {self.min_position:.2f} mm")

            # Home in positive direction to find maximum mechanical position
            if not self.home(direction=1):
                raise RuntimeError("Homing in positive direction failed.")
            time.sleep(1.0)
            self.max_position = self.get_position()
            self.logger.info(f"Maximum mechanical position: {self.max_position:.2f} mm")

            self.logger.info("Calibration complete.")
            return True

        except Exception as e:
            self.logger.error(f"Calibration failed: {e}")
            return False
        

    def move_to_width(self, width, speed=None):
        cmd = f"MOVE({width}" + (f",{speed})" if speed else ")")
        return self._ack_and_fin(cmd)

    def grip(self, force=None, width=None, speed=None):
        if force and width and speed:
            cmd = f"GRIP({force},{width},{speed})"
        elif force and width:
            cmd = f"GRIP({force},{width})"
        elif force:
            cmd = f"GRIP({force})"
        else:
            cmd = "GRIP()"
        return self._ack_and_fin(cmd)

    def release(self, distance=None, speed=None):
        if distance and speed:
            cmd = f"RELEASE({distance},{speed})"
        elif distance:
            cmd = f"RELEASE({distance})"
        else:
            cmd = "RELEASE()"
        return self._ack_and_fin(cmd)

    def stop(self):
        return self._ack_only("STOP()")

    def fast_stop(self):
        return self._ack_only("FASTSTOP()")

    def acknowledge_fast_stop(self):
        return self._ack_only("FSACK()")

    # Parameters
    def set_part_width_tolerance(self, value):
        return self.send_cmd(f"PWT={value}")

    def get_part_width_tolerance(self):
        return float(self.send_cmd("PWT?").split('=')[1])

    def set_clamping_travel(self, value):
        return self.send_cmd(f"CLT={value}")

    def get_clamping_travel(self):
        return float(self.send_cmd("CLT?").split('=')[1])

    # Status Queries
    def get_position(self):
        return float(self.send_cmd("POS?").split('=')[1])

    def get_speed(self):
        return float(self.send_cmd("SPEED?").split('=')[1])

    def get_force(self):
        return float(self.send_cmd("FORCE?").split('=')[1])

    def get_state(self):
        return int(self.send_cmd("GRIPSTATE?").split('=')[1])

    def get_temperature(self):
        return float(self.send_cmd("TEMP?").split('=')[1])

    def get_device_type(self):
        return self.send_cmd("DEVTYPE?").split('=')[1]

    def get_firmware_version(self):
        return self.send_cmd("VERSION?").split('=')[1]

    def get_serial_number(self):
        return self.send_cmd("SN?").split('=')[1]

    def get_device_tag(self):
        return self.send_cmd("TAG?").split('=')[1]

    def get_system_flags(self):
        return self.send_cmd("SYSFLAGS?")

    def get_system_flag(self, index):
        return bool(int(self.send_cmd(f"SYSFLAGS[{index}]?").split('=')[1]))

    def get_gripper_statistics(self):
        return list(map(int, self.send_cmd("GRIPSTATS?")[11:-1].split(',')))

    def tare_force_sensor(self, index=None):
        return self._ack_only(f"TARE({index})" if index is not None else "TARE()")

    def get_finger_data(self, index=None):
        return self.send_cmd(f"FDATA[{index}]?" if index is not None else "FDATA?")

    def get_finger_type(self, index=None):
        return self.send_cmd(f"FTYPE[{index}]?" if index is not None else "FTYPE?")

    def get_finger_flags(self, index=None):
        return self.send_cmd(f"FFLAGS[{index}]?" if index is not None else "FFLAGS?")

    def enable_autosend(self, name, interval_ms, delta_or_change=None):
        if isinstance(delta_or_change, bool):
            cmd = f"AUTOSEND(\"{name}\",{interval_ms},{str(delta_or_change).lower()})"
        elif isinstance(delta_or_change, (int, float)):
            cmd = f"AUTOSEND(\"{name}\",{interval_ms},{delta_or_change})"
        else:
            cmd = f"AUTOSEND(\"{name}\",{interval_ms})"
        return self._ack_only(cmd)

   # Internal helpers
    def _ack_and_fin(self, cmd, timeout=5.0):
        self.send(cmd)
        ack_received = False
        start = time.time()
        while time.time() - start < timeout:
            resp = self.recv()
            if resp.startswith("ACK"):
                ack_received = True
            elif resp.startswith("FIN"):
                return True
            elif resp.startswith("ERR"):
                error_code = self._parse_error_code(resp)
                raise RuntimeError(f"Gripper error {error_code}: {ERROR_CODES.get(error_code, 'Unknown error')}")
        if not ack_received:
            raise TimeoutError("No ACK received")
        raise TimeoutError("No FIN received within timeout")

    def _ack_only(self, cmd, timeout=2.0):
        self.send(cmd)
        start = time.time()
        while time.time() - start < timeout:
            resp = self.recv()
            if resp.startswith("ACK"):
                return True
            elif resp.startswith("ERR"):
                error_code = self._parse_error_code(resp)
                raise RuntimeError(f"Gripper error {error_code}: {ERROR_CODES.get(error_code, 'Unknown error')}")
        raise TimeoutError("No ACK received")

    def _parse_error_code(self, resp):
        try:
            parts = resp.split()
            return int(parts[2]) if len(parts) >= 3 else -1
        except (IndexError, ValueError):
            return -1
