#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ros2_jodell_epg40_050_gripper_rs485_msgs.srv import SetInt32, Trigger, ChangeBaudRate, SetSlaveID, SetPreset, GetAllPresets
from ros2_jodell_epg40_050_gripper_rs485_msgs.msg import GripperCommand
import serial
import struct
import time
from typing import Optional
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import sys
import json

class JodellGripper:
    def __init__(self, logger=None):
        self.port = None
        self.baud = None
        self.slave_id = None

        self.position = 128
        self.speed = 255
        self.force = 255
        self.brake = False
        self.connected = False
        self.ser = None
        self.logger = logger
        self.checker_activation = True

        self.reconnection_time = 3
        self.homing_time = 1.0
        self.homing_pos = 0
        self.homing_speed = 255
        self.homing_force = 255

        self.auto_hold_pos = 0
        self.auto_hold_speed = 50
        self.auto_hold_force = 50

        
        self.presets = {}

        self.REGISTERS = {
            'CONTROL': 0x03E8,
            'POSITION': 0x03E9,
            'SPEED_FORCE': 0x03EA,
            'ENCODER': 0x03FB,
            'BRAKE': 0x03FC,
            'STATUS': 0x07D0,
            'SOFTWARE_VERSION': 0x138C,
            'DROP_THRESHOLD': 0x1399,
            'Device_communication_configuration_register': 0x138E,
        }
        self.load_defaults()
        self.connect_gripper()

        
        

    def log_info(self, msg: str):
        if self.logger:
            self.logger.info(msg)

    def log_warn(self, msg: str):
        if self.logger:
            self.logger.warn(msg)

    def log_error(self, msg: str):
        if self.logger:
            self.logger.error(msg)
            
    def load_defaults(self):
        try:
            package_dir = get_package_share_directory(
                'ros2_jodell_epg40_050_gripper_rs485_interface'
            )
            config_path = os.path.join(package_dir, 'config', 'jodell_epg40_050_params.yaml')

            with open(config_path, 'r') as f:
                yaml_file_params = yaml.safe_load(f)

            config = yaml_file_params['epg40_050_config']

            self.port = config.get('port', '/dev/ttyACM0')
            self.baud = config.get('baud', 115200)

            self.slave_id = int(config.get('slave_id', 9))

            self.presets = config.get('presets', {})

        except Exception as e:
            self.log_error(f"⚠️ Failed to load defaults: {e}")
            self.port, self.baud, self.slave_id = "/dev/ttyACM0", 115200, hex(9)

    def connect_gripper(self):
        try:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except Exception:
                    pass

            self.ser = serial.Serial(self.port, self.baud, timeout=0.5)

            # quick probe: read STATUS register and check for a position byte
            resp = self._raw_read_register(self.slave_id, self.REGISTERS['STATUS'], 4)
            if resp and len(resp) >= 11:
                if resp[0] == self.slave_id:
                    try:
                        pos = resp[5]
                        if 0 <= pos <= 255:
                            self.connected = True
                            self.log_info(f"✅ Connected and verified (position={pos}) on {self.port} @ {self.baud} @ Slave ID={self.slave_id}")
                            return
                    except Exception:
                        pass
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.connected = False
            self.log_error(f"❌ Opened serial on {self.port} @ {self.baud} but failed to verify status")

        except serial.SerialException:
            self.ser = None
            self.connected = False
            self.log_error(f"❌ Failed to open serial on {self.port} @ {self.baud}")

    def add_crc(self, frame):
        crc = 0xFFFF
        for pos in frame:
            crc ^= pos
            for _ in range(8):
                if crc & 1:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        frame.extend(struct.pack("<H", crc))
        return frame

    def _raw_read_register(self, slave_id, reg_addr, count):
        try:
            if not self.ser or not self.ser.is_open:
                return None
            frame = bytearray([
                slave_id, 0x03,
                (reg_addr >> 8) & 0xFF,
                reg_addr & 0xFF,
                (count >> 8) & 0xFF if count > 0xFF else 0x00,
                count & 0xFF
            ])
            self.ser.reset_input_buffer()
            self.ser.write(self.add_crc(frame.copy()))
            time.sleep(0.02)
            resp = self.ser.read(64)
            return resp
        except Exception:
            return None

    

    def reconnect_gripper(self):
        while True:
            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()

                self.ser = serial.Serial(self.port, self.baud, timeout=0.5)
                # After opening, verify by reading status
                resp = self._raw_read_register(self.slave_id, self.REGISTERS['STATUS'], 4)
                if resp and len(resp) >= 11 and resp[0] == self.slave_id:
                    try:
                        pos = resp[5]
                        if 0 <= pos <= 255:
                            self.connected = True
                            self.log_info(f"✅ Connected to Jodell EPG40-050 Gripper on {self.port} @ {self.baud} @ slave ID={self.slave_id} @ (pos={pos})")
                            self.deactivate()
                            self.initialize()
                            break  # Exit loop once verified
                    except Exception:
                        pass

                # verification failed — close and retry
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                self.connected = False
                self.log_warn(f"❌ Opened serial but status probe failed on {self.port} @ {self.baud}, retrying...")
                time.sleep(1)

            except serial.SerialException:
                self.ser = None
                self.connected = False
                self.log_error(f"❌ Failed to connect to gripper on {self.port} @ {self.baud}, retrying...")
                time.sleep(1)  # wait before retrying
       

    def write_register(self, register_addr, data):
        """Write to Modbus register"""
        frame = bytearray([0x09, 0x10, (register_addr >> 8) & 0xFF, register_addr & 0xFF,
                          0x00, 0x01, 0x02,  # Write 1 register (2 bytes)
                          (data >> 8) & 0xFF, data & 0xFF])
        
        return self.send_command(frame, f"Write reg 0x{register_addr:04X}")

    def update_config_files(self, new_baudrate):
        package_share_dir = get_package_share_directory('ros2_jodell_epg40_050_gripper_rs485_interface')

        # Install-space config file
        install_path = os.path.join(package_share_dir, 'config', 'jodell_epg40_050_params.yaml')

        # Source-space config file
        workspace_root = os.path.realpath(os.path.join(package_share_dir, "../../../.."))
        src_path = os.path.join(
            workspace_root,
            "src",
            "ros2_jodell_epg40_050_gripper_rs485_interface",
            "config",
            "jodell_epg40_050_params.yaml"
        )

        for path in [install_path, src_path]:
            try:
                # Load existing YAML
                with open(path, 'r') as f:
                    config_data = yaml.safe_load(f)

                if "epg40_050_config" not in config_data:
                    config_data["epg40_050_config"] = {}

                # Update baud only
                config_data["epg40_050_config"]["baud"] = new_baudrate

                # Save back
                with open(path, 'w') as f:
                    yaml.safe_dump(config_data, f, sort_keys=False)

                self.log_info(f"[INFO] Updated baudrate to {new_baudrate} in: {path}")
                self.connected = False

            except Exception as e:
                self.log_error(f"[ERROR] Failed to update {path}: {e}")

        self.reconnect_gripper()


    def change_baud_rate(self, new_baud_rate):
        baud_codes = {
            115200: 0,
            57600: 1,
            38400: 2,
            19200: 3,
            9600: 4,
            4800: 5
        }
        
        if new_baud_rate not in baud_codes:
            return False, f"❌ Unsupported baud rate: {new_baud_rate}. " \
                        f"✅ Supported: {', '.join(map(str, baud_codes))}"
        baud_code = baud_codes[new_baud_rate]
        # Send baud rate change command
        response = self.write_register(self.REGISTERS['Device_communication_configuration_register'], baud_code)
        

        if response and len(response) >= 8:
            if response[0] == self.slave_id and response[1] == 0x10:
                self.baud = new_baud_rate
                self.update_config_files(new_baud_rate)
                return True, f"✅ Baud rate changed to {new_baud_rate}. ⚠️ Power cycle required"
        
        return False, f"❌ Failed to change baud rate to {new_baud_rate}"


    def update_config_slave_id(self, new_slave_id: int):
        """Update slave_id in YAML configs (both install + src)"""
        package_share_dir = get_package_share_directory('ros2_jodell_epg40_050_gripper_rs485_interface')

        install_path = os.path.join(package_share_dir, 'config', 'jodell_epg40_050_params.yaml')
        workspace_root = os.path.realpath(os.path.join(package_share_dir, "../../../.."))
        src_path = os.path.join(
            workspace_root,
            "src",
            "ros2_jodell_epg40_050_gripper_rs485_interface",
            "config",
            "jodell_epg40_050_params.yaml"
        )

        for path in [install_path, src_path]:
            try:
                with open(path, 'r') as f:
                    config_data = yaml.safe_load(f)

                if "epg40_050_config" not in config_data:
                    config_data["epg40_050_config"] = {}

                config_data["epg40_050_config"]["slave_id"] = new_slave_id

                with open(path, 'w') as f:
                    yaml.safe_dump(config_data, f, sort_keys=False)

                self.log_info(f"[INFO] Updated slave_id to {new_slave_id} in: {path}")
            except Exception as e:
                self.log_error(f"[ERROR] Failed to update {path}: {e}")

    def find_slave_id(self):
        """Scan Modbus addresses (1–247) to detect active gripper ID"""
        reg_addr = 0x138D  # Slave ID register
        for slave_id in range(1, 248):
            resp = None
            try:
                # try open serial if needed
                if not self.ser or not (self.ser and self.ser.is_open):
                    try:
                        self.ser = serial.Serial(self.port, self.baud, timeout=0.5)
                    except Exception:
                        continue
                resp = self._raw_read_register(slave_id, reg_addr, 1)
            except Exception:
                resp = None

            if resp and len(resp) >= 5 and resp[0] == slave_id:
                self.log_info(f"✅ Found gripper at slave ID {slave_id}")
                return slave_id
        self.log_error("❌ No gripper detected on any slave ID (1–247)")
        return None

    def change_slave_id(self, new_slave_id: int):
        """Send Modbus command to change slave ID"""
        if not (1 <= new_slave_id <= 247):
            return False, f"❌ Invalid slave ID: {new_slave_id} (must be 1–247)"

        # Try to auto-detect current ID if unknown
        if not self.slave_id:
            self.slave_id = self.find_slave_id()
            if not self.slave_id:
                return False, "❌ Could not detect current gripper slave ID"

        reg_addr = 0x138D
        frame = bytearray([
            self.slave_id, 0x06,
            (reg_addr >> 8) & 0xFF, reg_addr & 0xFF,
            0x00, new_slave_id
        ])
        self.send_command(frame, f"Change slave ID to {new_slave_id}")

        detected_id = self.find_slave_id()
        if detected_id == new_slave_id:
            old_id = self.slave_id
            self.slave_id = new_slave_id
            self.update_config_slave_id(new_slave_id)
            self.reconnect_gripper()
            
            return True, f"✅ Slave ID changed from {old_id} → {new_slave_id}"
        else:
            return False, f"❌ Failed to confirm slave ID {new_slave_id}"

    def send_command(self, frame, description=""):
        if not self.ser or not (self.ser and self.ser.is_open):
            return None
        try:
            if description:
                self.log_info(f"🔧 {description}")
            crc_frame = self.add_crc(frame.copy())
            self.ser.write(crc_frame)
            time.sleep(0.01)
            response = self.ser.read(64)
            return response
        except (serial.SerialException, OSError):
            self.connected = False
            self.log_error("❌ Connection lost while sending command")
            return None

    def initialize(self):
        if not self.connected:
            return
        reg = self.REGISTERS['CONTROL']
        # Clear faults
        frame = bytearray([self.slave_id, 0x10, (reg >> 8) & 0xFF, reg & 0xFF, 0x00, 0x01, 0x02, 0x00, 0x00])
        self.send_command(frame, "Clear faults")
        time.sleep(0.1)

        # Activate gripper
        frame = bytearray([self.slave_id, 0x10, (reg >> 8) & 0xFF, reg & 0xFF, 0x00, 0x01, 0x02, 0x00, 0x01])
        self.send_command(frame, "Activate gripper")
        time.sleep(self.homing_time)
        self.set_parameters(self.homing_pos, self.homing_speed, self.homing_force)
        self.execute_move()
        self.log_info("Gripper Home Position reached")

    def get_software_version(self) -> Optional[str]:
        if not self.connected:
            return None
        reg = self.REGISTERS['SOFTWARE_VERSION']
        resp = self._raw_read_register(self.slave_id, reg, 1)
        if resp and len(resp) >= 5 and resp[0] == self.slave_id:
            version_data = int.from_bytes(resp[3:5], byteorder='big')
            return f"{(version_data >> 8) & 0xFF}.{version_data & 0xFF}"
        return None

    def set_drop_threshold(self, threshold: int) -> bool:
        if not self.connected:
            return False
        reg = self.REGISTERS['DROP_THRESHOLD']
        data = threshold.to_bytes(2, byteorder='big')
        frame = bytearray([self.slave_id, 0x10, (reg >> 8) & 0xFF, reg & 0xFF, 0x00, 0x01, 0x02])
        frame.extend(data)
        resp = self.send_command(frame, f"Set drop threshold {threshold}")
        return resp is not None

    def activate(self) -> bool:
        if not self.connected:
            return False
        reg = self.REGISTERS['CONTROL']
        data = (0x0001).to_bytes(2, 'big')
        frame = bytearray([self.slave_id, 0x10, (reg >> 8) & 0xFF, reg & 0xFF, 0x00, 0x01, 0x02])
        frame.extend(data)
        resp = self.send_command(frame, "Activate gripper")
        return resp is not None

    def deactivate(self) -> bool:
        if not self.connected:
            return False
        reg = self.REGISTERS['CONTROL']
        data = (0x0000).to_bytes(2, 'big')
        frame = bytearray([self.slave_id, 0x10, (reg >> 8) & 0xFF, reg & 0xFF, 0x00, 0x01, 0x02])
        frame.extend(data)
        resp = self.send_command(frame, "Deactivate gripper")
        return resp is not None

    def set_parameters(self, position=None, speed=None, force=None):
        if position is not None:
            self.position = max(0, min(255, position))
        if speed is not None:
            self.speed = max(0, min(255, speed))
        if force is not None:
            self.force = max(0, min(255, force))

    def set_brake(self, force_open: bool):
        self.brake = force_open
        if not self.connected:
            return
        reg = self.REGISTERS['BRAKE']
        value = 0x01 if force_open else 0x00
        frame = bytearray([self.slave_id, 0x10, (reg >> 8) & 0xFF, reg & 0xFF, 0x00, 0x01, 0x02, 0x00, value])
        self.send_command(frame, f"{'Force brake open' if force_open else 'Reset brake'}")

    def read_status(self):
        """High-level status read (requires connected). Returns parsed dict or None."""
        if not self.connected:
            return None
        resp = self._raw_read_register(self.slave_id, self.REGISTERS['STATUS'], 4)
        if resp and len(resp) >= 11:
            return {'position_status': resp[5], 'brake': self.brake}
        return None

    def execute_move(self):
        reg = self.REGISTERS['CONTROL']
        params = bytes([0x06, 0x00, self.slave_id, self.position, 0x00, self.speed, self.force])
        frame = bytearray([self.slave_id, 0x10, (reg >> 8) & 0xFF, reg & 0xFF, 0x00, 0x03])
        frame.extend(params)
        self.send_command(frame, "Move command")

    def preset_mode(self, preset_number: int) -> str:
        if not self.presets:
            self.log_warn("⚠️ No presets loaded from YAML")
            return False

        if str(preset_number) not in self.presets:
            self.log_warn(f"❌ Invalid preset number: {preset_number}")
            return False

        preset = self.presets[str(preset_number)]
        name = preset.get('name', f"Preset {preset_number}")
        pos = preset.get('pos', self.position)
        spd = preset.get('spd', self.speed)
        frc = preset.get('frc', self.force)

        msg = f"🎯 Executing preset: {name} (pos={pos}, spd={spd}, frc={frc})"
        self.set_parameters(pos, spd, frc)
        self.execute_move()
        return msg

    def close_connection(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.connected = False


class GripperNode(Node):
    def __init__(self):
        super().__init__('jodell_gripper_node')
        self.gripper = JodellGripper(self.get_logger())
        self.gripper.initialize()

        self.status_pub = self.create_publisher(String, 'jodell_epg40_050/gripper_status', 10)
        self.conn_pub = self.create_publisher(String, 'jodell_epg40_050/connection_status', 10)

        self.create_subscription(GripperCommand, 'jodell_epg40_050/gripper_cmd', self.cmd_callback, 10)

        # ROS 2 services
        self.create_service(Trigger, 'jodell_epg40_050/activate_gripper', self.activate_callback)
        self.create_service(Trigger, 'jodell_epg40_050/deactivate_gripper', self.deactivate_callback)
        self.create_service(SetInt32, 'jodell_epg40_050/set_drop_threshold', self.set_drop_threshold_callback)
        self.create_service(SetInt32, 'jodell_epg40_050/preset_mode', self.preset_mode_callback)
        self.baud_service = self.create_service(ChangeBaudRate,'jodell_epg40_050/change_baud_rate',self.change_baud_rate_callback)
        self.srv_set_slave = self.create_service(SetSlaveID, "jodell_epg40_050/set_slave_id",self.set_slave_id_callback)
        self.srv_set_preset = self.create_service(SetPreset, "jodell_epg40_050/set_preset", self.set_preset_callback)
        self.get_all_presets_srv = self.create_service(GetAllPresets, 'jodell_epg40_050/get_all_presets', self.get_all_presets_callback)

        self.get_logger().info("🚀 Gripper node started")

        self.get_logger().info(f"Connected to Port : {self.gripper.port}")
        self.get_logger().info(f"Connected to Baud : {self.gripper.baud}")
        self.get_logger().info(f"Connected to Slave_ID : {self.gripper.slave_id}")

        # Auto-stop
        self.auto_stop_enabled = False
        self.last_position = None
        self.last_movement_time = self.get_clock().now()
        self.position_stop_threshold_time = 0.01

        # Timers
        self.create_timer(0.01, self.auto_stop_check)
        self.create_timer(0.1, self.publish_status)
        self.create_timer(1.0, self.publish_connection_status)

        self.auto_stop_check_brake = ""

    # Command topic callback
    def cmd_callback(self, msg: GripperCommand):
        if not self.gripper.connected:
            self.get_logger().warn("Gripper not connected. Ignoring command.")
            return

        if self.gripper.checker_activation != True:
            self.get_logger().warn("Gripper is not activated. Ignoring command.")
            return

        # Always at least pos, spd, frc
        if msg.position is not None and msg.speed is not None and msg.force is not None:
            self.gripper.set_brake(False)
            pos, spd, frc = msg.position, msg.speed, msg.force
            self.gripper.set_parameters(pos, spd, frc)

            # Mode handling (instead of msg.data[3])
            if msg.mode in ["hold", "free", "continuous"]:
                self.auto_stop_check_brake = msg.mode
            else:
                self.auto_stop_check_brake = "continuous"

            self.gripper.set_brake(True)
            self.gripper.execute_move()

            self.auto_stop_enabled = True
            self.last_position = None
            self.last_movement_time = self.get_clock().now()
        else:
            self.get_logger().error(
                "Invalid command: expected at least [position, speed, force]"
            )

    # Auto-stop logic
    def auto_stop_check(self):
        if not self.auto_stop_enabled or not self.gripper.connected:
            return

        if self.auto_stop_check_brake == "continuous":
            return

        status = self.gripper.read_status()
        if not status:
            return

        current_pos = status["position_status"]

        if self.last_position is None:
            self.last_position = current_pos
            self.last_movement_time = self.get_clock().now()
            return

        if abs(current_pos - self.last_position) > 1:
            self.last_position = current_pos
            self.last_movement_time = self.get_clock().now()
        else:
            duration = (
                (self.get_clock().now() - self.last_movement_time).nanoseconds / 1e9
            )
            if duration >= self.position_stop_threshold_time:
                self.get_logger().info(
                    f"✋ Auto-stop at {current_pos} for {duration:.2f}s"
                )
                self.gripper.set_brake(False)
                self.gripper.set_parameters(
                    position=current_pos + self.gripper.auto_hold_pos, speed=self.gripper.auto_hold_speed, force=self.gripper.auto_hold_force
                )
                self.gripper.execute_move()

                if self.auto_stop_check_brake == "free":
                    self.gripper.set_brake(False)
                    self.get_logger().info("Gripper brake released, jaws free")
                elif self.auto_stop_check_brake == "hold":
                    self.gripper.set_brake(True)
                    self.get_logger().info("Gripper brake engaged, Jaws lock")

                self.gripper.execute_move()
                self.auto_stop_enabled = False

    def change_baud_rate_callback(self, request, response):
        success, msg = self.gripper.change_baud_rate(request.baud_rate)
        response.success = success
        response.message = msg
        return response
    
    def set_slave_id_callback(self, request, response):
        success, msg = self.gripper.change_slave_id(request.slave_id)
        response.success = success
        response.message = msg
        return response


    # Status publisher
    def publish_status(self):
        if not self.gripper.connected:
            return
        status = self.gripper.read_status()
        if status:
            msg = String()
            msg.data = str(status)
            self.status_pub.publish(msg)

    # Connection + software version publisher
    def publish_connection_status(self):
        if not self.gripper.connected:
            self.gripper.connect_gripper()
            if self.gripper.connected:
                self.gripper.initialize()

        msg = String()
        version = self.gripper.get_software_version() or "Unknown"

        # Connection info dictionary
        conn_status = {
            'baud': self.gripper.baud,
            'slave_id': self.gripper.slave_id,
            'port': self.gripper.port,
            'connected': 'Yes' if self.gripper.connected else 'No',
            'software_version': version
        }

        # Track healthy connection
        now = time.time()
        if version != "Unknown":
            self.last_ok_time = now
        else:
            # If "Unknown" for more than 5 seconds → mark disconnected
            if hasattr(self, "last_ok_time") and (now - self.last_ok_time > self.gripper.reconnection_time):
                self.gripper.connected = False
                conn_status['connected'] = 'No'

        msg.data = str(conn_status)
        self.conn_pub.publish(msg)

    # Service callbacks
    def activate_callback(self, request, response):

            self.gripper.checker_activation = True
            success = self.gripper.activate()
            response.success = success
            response.message = "Activated" if success else "Failed"
            return response

    def deactivate_callback(self, request, response):
        self.gripper.checker_activation = False
        success = self.gripper.deactivate()
        response.success = success
        response.message = "Deactivated" if success else "Failed"
        return response

    def set_drop_threshold_callback(self, request, response):
        success = self.gripper.set_drop_threshold(request.data)
        response.success = success
        response.message = f"Drop threshold set to {request.data}" if success else "Failed"
        return response

    def preset_mode_callback(self, request, response):
        msg = self.gripper.preset_mode(request.data)
        if msg:
            response.success = True
            response.message = msg   # return the full execution message
        else:
            response.success = False
            response.message = f"❌ Invalid preset: {request.data}"
        return response

    def set_preset_callback(self, request, response):
        preset_id = str(request.preset_id)
        name = request.name
        pos, spd, frc = request.position, request.speed, request.force

        if not self.gripper.connected:
            response.success = False
            response.message = "❌ Gripper not connected"
            return response

        # Prepare preset dictionary
        preset_data = {
            "name": name,
            "pos": pos,
            "spd": spd,
            "frc": frc
        }

        # Update YAML in both install + src
        package_share_dir = get_package_share_directory(
            'ros2_jodell_epg40_050_gripper_rs485_interface'
        )
        install_path = os.path.join(package_share_dir, 'config', 'jodell_epg40_050_params.yaml')
        workspace_root = os.path.realpath(os.path.join(package_share_dir, "../../../.."))
        src_path = os.path.join(
            workspace_root,
            "src",
            "ros2_jodell_epg40_050_gripper_rs485_interface",
            "config",
            "jodell_epg40_050_params.yaml"
        )

        for path in [install_path, src_path]:
            try:
                with open(path, 'r') as f:
                    config_data = yaml.safe_load(f)

                if "epg40_050_config" not in config_data:
                    config_data["epg40_050_config"] = {}
                if "presets" not in config_data["epg40_050_config"]:
                    config_data["epg40_050_config"]["presets"] = {}

                config_data["epg40_050_config"]["presets"][preset_id] = preset_data

                with open(path, 'w') as f:
                    yaml.safe_dump(config_data, f, sort_keys=False)

                self.get_logger().info(f"✅ Saved preset {preset_id}: {preset_data} → {path}")

            except Exception as e:
                self.get_logger().error(f"[ERROR] Failed to update {path}: {e}")

        # Update in-memory presets
        self.gripper.presets[preset_id] = preset_data

        response.success = True
        response.message = f"✅ Preset {preset_id} ('{name}') saved and executed"
        return response

    def get_all_presets_callback(self, request, response):
        try:
            # Convert presets dict to JSON string
            response.presets_json = json.dumps(self.gripper.presets)
        except Exception as e:
            response.presets_json = f"{{'error': '{str(e)}'}}"
        return response

    
    def destroy_node(self):
        self.gripper.close_connection()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("👋 Shutting down gripper node cleanly...")

        # Then cleanup
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        sys.exit(0)  # clean exit on Ctrl+C
    except Exception as e:
        # Catch unexpected errors
        node.get_logger().error(f"❌ Unexpected error: {e}")

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        sys.exit(1)  # non-zero exit for errors
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()
