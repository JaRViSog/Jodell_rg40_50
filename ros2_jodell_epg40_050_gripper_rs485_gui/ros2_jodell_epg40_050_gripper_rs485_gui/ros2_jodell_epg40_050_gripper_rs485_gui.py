#!/usr/bin/env python3

import sys
import time
import queue
from dataclasses import dataclass

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QSpinBox, QComboBox, QTextEdit, QGroupBox, QMessageBox, QLineEdit, QCheckBox
)

# ROS imports optional at module import
try:
    from std_msgs.msg import String
    from ros2_jodell_epg40_050_gripper_rs485_msgs.msg import GripperCommand
    from ros2_jodell_epg40_050_gripper_rs485_msgs.srv import (
        ChangeBaudRate, Trigger, SetInt32, SetPreset, SetSlaveID, GetAllPresets
    )
except Exception:
    String = None
    GripperCommand = None
    ChangeBaudRate = Trigger = SetInt32 = SetPreset = SetSlaveID = GetAllPresets = None

@dataclass
class RosRequest:
    kind: str
    payload: dict

class RosWorker(QtCore.QThread):
    status_update = QtCore.pyqtSignal(str)
    conn_update = QtCore.pyqtSignal(str)
    service_result = QtCore.pyqtSignal(str, object)

    def __init__(self):
        super().__init__()
        self._queue = queue.Queue()
        self._stopping = False
        self.node = None

    def run(self):
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import String as RosString
            from ros2_jodell_epg40_050_gripper_rs485_msgs.msg import GripperCommand as RosGripperCommand
            from ros2_jodell_epg40_050_gripper_rs485_msgs.srv import (
                ChangeBaudRate as RosChangeBaudRate,
                Trigger as RosTrigger,
                SetInt32 as RosSetInt32,
                SetPreset as RosSetPreset,
                SetSlaveID as RosSetSlaveID
            )
        except Exception as e:
            self.status_update.emit(f"[ROS ERROR] failed to import rclpy or message types: {e}")
            return

        rclpy.init()
        node = Node('gripper_gui_node')
        self.node = node

        pub_cmd = node.create_publisher(RosGripperCommand, 'jodell_epg40_050/gripper_cmd', 10)

        def on_status(msg: RosString):
            self.status_update.emit(str(msg.data))

        def on_conn(msg: RosString):
            self.conn_update.emit(str(msg.data))

        node.create_subscription(RosString, 'jodell_epg40_050/gripper_status', on_status, 10)
        node.create_subscription(RosString, 'jodell_epg40_050/connection_status', on_conn, 10)

        clients = {}
        def make_client(srv_type, name):
            cli = node.create_client(srv_type, name)
            if not cli.wait_for_service(timeout_sec=1.0):
                self.service_result.emit(name, '[WARN] Service not available yet')
            return cli

        clients['activate'] = make_client(RosTrigger, 'jodell_epg40_050/activate_gripper')
        clients['deactivate'] = make_client(RosTrigger, 'jodell_epg40_050/deactivate_gripper')
        clients['change_baud'] = make_client(RosChangeBaudRate, 'jodell_epg40_050/change_baud_rate')
        clients['set_slave'] = make_client(RosSetSlaveID, 'jodell_epg40_050/set_slave_id')
        clients['preset_mode'] = make_client(RosSetInt32, 'jodell_epg40_050/preset_mode')
        clients['set_preset'] = make_client(RosSetPreset, 'jodell_epg40_050/set_preset')
        clients['set_drop'] = make_client(RosSetInt32, 'jodell_epg40_050/set_drop_threshold')
        clients['get_presets'] = make_client(GetAllPresets, 'jodell_epg40_050/get_all_presets')

        try:
            while rclpy.ok() and not self._stopping:
                rclpy.spin_once(node, timeout_sec=0.1)
                try:
                    req: RosRequest = self._queue.get_nowait()
                except queue.Empty:
                    continue

                try:
                    # publish move
                    if req.kind == 'publish_move':
                        gc = RosGripperCommand()
                        gc.position = int(req.payload.get('position', 128))
                        gc.speed = int(req.payload.get('speed', 255))
                        gc.force = int(req.payload.get('force', 255))
                        gc.mode = str(req.payload.get('mode', 'continuous'))
                        pub_cmd.publish(gc)
                        self.service_result.emit('publish_move', f"pos={gc.position} spd={gc.speed} frc={gc.force}")

                    # activate/deactivate
                    elif req.kind in ['activate','deactivate']:
                        cli = clients[req.kind]
                        if cli.wait_for_service(timeout_sec=1.0):
                            fut = cli.call_async(RosTrigger.Request())
                            rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
                            res = fut.result()
                            # Emit something like: "success=True msg=..."
                            self.service_result.emit(req.kind, f"success={getattr(res,'success',False)} msg={getattr(res,'message','')}")
                        else:
                            self.service_result.emit(req.kind, 'Service unavailable')

                    # change baud
                    elif req.kind == 'change_baud':
                        cli = clients['change_baud']
                        if cli.wait_for_service(timeout_sec=1.0):
                            r = RosChangeBaudRate.Request()
                            r.baud_rate = int(req.payload.get('baud', 115200))
                            fut = cli.call_async(r)
                            rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
                            res = fut.result()
                            self.service_result.emit('change_baud', f"success={getattr(res,'success',False)} msg={getattr(res,'message','')}")
                        else:
                            self.service_result.emit('change_baud', 'Service unavailable')

                    # set slave ID
                    elif req.kind == 'set_slave':
                        cli = clients['set_slave']
                        if cli.wait_for_service(timeout_sec=1.0):
                            r = RosSetSlaveID.Request()
                            r.slave_id = int(req.payload.get('slave_id', 9))
                            fut = cli.call_async(r)
                            rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
                            res = fut.result()
                            self.service_result.emit('set_slave', f"success={getattr(res,'success',False)} msg={getattr(res,'message','')}")
                        else:
                            self.service_result.emit('set_slave', 'Service unavailable')

                    # preset
                    elif req.kind in ['preset_mode','set_preset','set_drop']:
                        # handled similarly
                        cli_name = req.kind if req.kind in clients else None
                        cli = clients.get(cli_name)
                        if cli and cli.wait_for_service(timeout_sec=1.0):
                            if req.kind=='preset_mode':
                                r = RosSetInt32.Request()
                                r.data = int(req.payload.get('preset_id',1))
                            elif req.kind=='set_drop':
                                r = RosSetInt32.Request()
                                r.data = int(req.payload.get('threshold',0))
                            elif req.kind=='set_preset':
                                r = RosSetPreset.Request()
                                r.preset_id = int(req.payload.get('preset_id',1))
                                r.name = str(req.payload.get('name','preset'))
                                r.position = int(req.payload.get('position',128))
                                r.speed = int(req.payload.get('speed',255))
                                r.force = int(req.payload.get('force',255))
                            fut = cli.call_async(r)
                            rclpy.spin_until_future_complete(node,fut,timeout_sec=5.0)
                            res = fut.result()
                            self.service_result.emit(req.kind, f"success={getattr(res,'success',False)} msg={getattr(res,'message','')}")
                        else:
                            self.service_result.emit(req.kind,'Service unavailable')

                    elif req.kind == 'get_presets':
                        cli = clients.get('get_presets')
                        if cli and cli.wait_for_service(timeout_sec=1.0):
                            r = GetAllPresets.Request()  # empty request
                            fut = cli.call_async(r)
                            rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
                            res = fut.result()
                            import json
                            try:
                                presets_dict = json.loads(res.presets_json)
                                presets_list = []
                                for k, v in presets_dict.items():
                                    presets_list.append({
                                        'preset_id': int(k),
                                        'name': v.get('name',''),
                                        'position': v.get('pos',0),
                                        'speed': v.get('spd',0),
                                        'force': v.get('frc',0)
                                    })
                            except Exception as e:
                                presets_list = []
                                self.status_update.emit(f"Error parsing presets JSON: {e}")
                            self.service_result.emit('get_presets', presets_list)
                        else:
                            self.service_result.emit('get_presets', [])



                except Exception as e:
                    self.service_result.emit(req.kind,f"Exception: {e}")

        except Exception as e:
            self.status_update.emit(f"[ROS THREAD FAILED] {e}")

        # cleanup
        try:
            if node:
                node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass

    def enqueue(self, req: RosRequest):
        self._queue.put(req)

    def stop(self):
        self._stopping = True

# --------------------- GUI ---------------------

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Jodell Gripper GUI (ROS)')
        self.resize(1200, 800)  # larger window for MSI Katana
        self.setMinimumSize(1000, 700)
        self.setStyleSheet(self._dark_stylesheet())

        self.ros_worker = RosWorker()
        self.ros_worker.status_update.connect(self.on_status_update)
        self.ros_worker.conn_update.connect(self.on_conn_update)
        self.ros_worker.service_result.connect(self.on_service_result)

        # feature: auto_scroll default OFF
        self.auto_scroll = False

        self._build_ui()
        self.ros_worker.start()

    def _dark_stylesheet(self):
        return """
        QWidget {background-color:#1e1e2f; color:#ddd; font-size:14px;}
        QGroupBox {border:1px solid #444; border-radius:5px; margin-top:10px;}
        QGroupBox::title {subcontrol-origin: margin; subcontrol-position: top left; padding:0 3px;}
        QLabel {color:#ccc;}
        QPushButton {background-color:#3a3a5c; color:white; border-radius:5px; padding:5px;}
        QPushButton:hover {background-color:#505080;}
        QSpinBox, QComboBox, QLineEdit {background-color:#2e2e3f; color:white; border:1px solid #555;}
        QTextEdit {background-color:#252535; color:#00ff00; font-family: monospace;}
        """

    def _build_ui(self):
        v_main = QVBoxLayout()

        # Top connection labels + autoscroll checkbox
        h_top = QHBoxLayout()
        self.lbl_conn = QLabel('Connection: -')
        self.lbl_swver = QLabel('Software: -')
        h_top.addWidget(self.lbl_conn)
        h_top.addWidget(self.lbl_swver)

        # spacer and auto-scroll checkbox on right
        spacer = QtWidgets.QSpacerItem(20, 10, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        h_top.addItem(spacer)
        self.chk_autoscroll = QCheckBox('Auto-scroll')
        self.chk_autoscroll.setChecked(False)  # <-- start unchecked
        self.chk_autoscroll.stateChanged.connect(self.on_autoscroll_toggled)
        h_top.addWidget(self.chk_autoscroll)

        v_main.addLayout(h_top)

        # Controls group (first row)
        g_ctrl = QGroupBox('Controls')
        gh = QHBoxLayout()

        # Manual Move
        mv_box = QGroupBox('Manual Move')
        mvh = QHBoxLayout()
        self.spin_pos = QSpinBox(); self.spin_pos.setRange(0,255); self.spin_pos.setValue(128)
        self.spin_spd = QSpinBox(); self.spin_spd.setRange(0,255); self.spin_spd.setValue(255)
        self.spin_frc = QSpinBox(); self.spin_frc.setRange(0,255); self.spin_frc.setValue(255)
        self.cmb_mode = QComboBox(); self.cmb_mode.addItems(['continuous','hold','free'])
        btn_move = QPushButton('Publish Move'); btn_move.clicked.connect(self.publish_move)
        for lbl, widget in zip(['Pos','Spd','Frc','Mode'], [self.spin_pos,self.spin_spd,self.spin_frc,self.cmb_mode]):
            mvh.addWidget(QLabel(lbl)); mvh.addWidget(widget)
        mvh.addWidget(btn_move)
        mv_box.setLayout(mvh)

        # Activate/Deactivate
        act_box = QGroupBox('Activation')
        ah = QVBoxLayout()
        self.btn_act = QPushButton('Activate'); self.btn_deact = QPushButton('Deactivate')
        self.btn_act.clicked.connect(lambda: self.enqueue_request('activate', {}))
        self.btn_deact.clicked.connect(lambda: self.enqueue_request('deactivate', {}))
        ah.addWidget(self.btn_act); ah.addWidget(self.btn_deact)

        # status light (starts green)
        self.lbl_act_status = QLabel('')
        self.lbl_act_status.setFixedSize(16,16)
        self._set_activation_led('on')   # <-- start as green on GUI start
        ah.addWidget(self.lbl_act_status)
        act_box.setLayout(ah)

        # Baud/Slave
        bs_box = QGroupBox('Baud / Slave')
        bsh = QHBoxLayout()
        self.cmb_baud = QComboBox(); self.cmb_baud.addItems(['115200','57600','38400','19200','9600','4800'])
        btn_baud = QPushButton('Change Baud'); btn_baud.clicked.connect(self.change_baud)
        self.spin_slave = QSpinBox(); self.spin_slave.setRange(1,247); self.spin_slave.setValue(9)
        btn_set_slave = QPushButton('Set Slave ID'); btn_set_slave.clicked.connect(self.set_slave)
        bsh.addWidget(QLabel('Baud')); bsh.addWidget(self.cmb_baud); bsh.addWidget(btn_baud)
        bsh.addWidget(QLabel('Slave')); bsh.addWidget(self.spin_slave); bsh.addWidget(btn_set_slave)
        bs_box.setLayout(bsh)

        # Drop Threshold
        dt_box = QGroupBox('Drop Threshold')
        dth = QHBoxLayout()
        self.spin_drop = QSpinBox(); self.spin_drop.setRange(0,65535); self.spin_drop.setValue(0)
        btn_set_drop = QPushButton('Set Drop Threshold'); btn_set_drop.clicked.connect(self.set_drop)
        dth.addWidget(self.spin_drop); dth.addWidget(btn_set_drop)
        dt_box.setLayout(dth)

        # Add controls to first row
        for box in [mv_box, act_box, bs_box, dt_box]:
            gh.addWidget(box)
        g_ctrl.setLayout(gh)
        v_main.addWidget(g_ctrl)

        # ---------- Second row: Presets ----------
        pr_box = QGroupBox('Presets')
        prh = QHBoxLayout()
        self.le_preset_id = QSpinBox(); self.le_preset_id.setRange(1,99); self.le_preset_id.setValue(1)
        self.le_preset_name = QLineEdit(); self.le_preset_name.setPlaceholderText('Name')
        self.spin_p_pos = QSpinBox(); self.spin_p_pos.setRange(0,255); self.spin_p_pos.setValue(128)
        self.spin_p_spd = QSpinBox(); self.spin_p_spd.setRange(0,255); self.spin_p_spd.setValue(255)
        self.spin_p_frc = QSpinBox(); self.spin_p_frc.setRange(0,255); self.spin_p_frc.setValue(255)
        btn_run_preset = QPushButton('Run Preset'); btn_run_preset.clicked.connect(self.run_preset)
        btn_set_preset = QPushButton('Save Preset'); btn_set_preset.clicked.connect(self.save_preset)
        btn_fetch_presets = QPushButton("Fetch Presets"); btn_fetch_presets.clicked.connect(lambda: self.enqueue_request('get_presets', {}))

        for lbl, widget in zip(['ID','Name','Pos','Spd','Frc'], [self.le_preset_id,self.le_preset_name,self.spin_p_pos,self.spin_p_spd,self.spin_p_frc]):
            prh.addWidget(QLabel(lbl)); prh.addWidget(widget)
        prh.addWidget(btn_run_preset); prh.addWidget(btn_set_preset); prh.addWidget(btn_fetch_presets)
        pr_box.setLayout(prh)
        v_main.addWidget(pr_box)

        # Presets Table
        self.tbl_presets = QtWidgets.QTableWidget()
        self.tbl_presets.setColumnCount(5)
        self.tbl_presets.setHorizontalHeaderLabels(['ID','Name','Pos','Spd','Frc'])
        self.tbl_presets.horizontalHeader().setStretchLastSection(True)
        v_main.addWidget(self.tbl_presets)

        # Log
        self.txt_log = QTextEdit(); self.txt_log.setReadOnly(True); self.txt_log.setLineWrapMode(QTextEdit.NoWrap)
        v_main.addWidget(QLabel('Status topic (raw)'))
        v_main.addWidget(self.txt_log)

        self.setLayout(v_main)


    # ---------- GUI actions ----------
    def enqueue_request(self, kind, payload):
        self.ros_worker.enqueue(RosRequest(kind, payload))
        self.log(f"Enqueued {kind} {payload}")

    def publish_move(self):
        self.enqueue_request('publish_move',{
            'position': self.spin_pos.value(),
            'speed': self.spin_spd.value(),
            'force': self.spin_frc.value(),
            'mode': self.cmb_mode.currentText()
        })

    def change_baud(self):
        self.enqueue_request('change_baud',{'baud':int(self.cmb_baud.currentText())})

    def set_slave(self):
        self.enqueue_request('set_slave',{'slave_id':self.spin_slave.value()})

    def run_preset(self):
        self.enqueue_request('preset_mode',{'preset_id':self.le_preset_id.value()})

    def save_preset(self):
        self.enqueue_request('set_preset',{
            'preset_id': self.le_preset_id.value(),
            'name': self.le_preset_name.text() or f"preset_{self.le_preset_id.value()}",
            'position': self.spin_p_pos.value(),
            'speed': self.spin_p_spd.value(),
            'force': self.spin_p_frc.value()
        })

    def set_drop(self):
        self.enqueue_request('set_drop',{'threshold':self.spin_drop.value()})

    def _set_activation_led(self, state: str):
        # states: 'on' (green), 'off' (red), 'unknown' (gray)
        if state == 'on':
            self.lbl_act_status.setStyleSheet('background-color: #00cc44; border-radius: 8px;')
        elif state == 'off':
            self.lbl_act_status.setStyleSheet('background-color: #cc0000; border-radius: 8px;')
        else:
            self.lbl_act_status.setStyleSheet('background-color: #ff9900; border-radius: 8px;')

    @QtCore.pyqtSlot(str)
    def on_status_update(self, txt):
        self.log(f"STATUS > {txt}")
        try:
            obj = eval(txt) if txt.startswith('{') else None
            if obj and isinstance(obj, dict):
                sw = obj.get('software_version') or obj.get('software')
                if sw: self.lbl_swver.setText(f"Software: {sw}")
        except Exception: pass

    @QtCore.pyqtSlot(str)
    def on_conn_update(self, txt):
        self.log(f"CONN > {txt}")
        try:
            obj = eval(txt) if txt.startswith('{') else None
            if obj and isinstance(obj, dict):
                ok = obj.get('connected'); port = obj.get('port'); baud = obj.get('baud')
                slave = obj.get('slave_id') or obj.get('slave')
                self.lbl_conn.setText(f"Connected: {ok} | port={port} baud={baud} slave={slave}")
        except Exception: pass

    @QtCore.pyqtSlot(str, object)
    def on_service_result(self, svc, res):
        if svc == 'get_presets':
            try:
                presets = res  # now it is a Python list of dicts
                self.tbl_presets.setRowCount(0)
                if not presets:
                    self.log("No presets found.")
                for p in presets:
                    row = self.tbl_presets.rowCount()
                    self.tbl_presets.insertRow(row)
                    self.tbl_presets.setItem(row,0,QtWidgets.QTableWidgetItem(str(p.get('preset_id',''))))
                    self.tbl_presets.setItem(row,1,QtWidgets.QTableWidgetItem(str(p.get('name',''))))
                    self.tbl_presets.setItem(row,2,QtWidgets.QTableWidgetItem(str(p.get('position',''))))
                    self.tbl_presets.setItem(row,3,QtWidgets.QTableWidgetItem(str(p.get('speed',''))))
                    self.tbl_presets.setItem(row,4,QtWidgets.QTableWidgetItem(str(p.get('force',''))))
            except Exception as e:
                self.log(f"Error updating presets table: {e}")
            return

        # activation/deactivation: set LED
        if svc in ['activate','deactivate']:
            # res is expected as a string like "success=True msg=..." or an error string
            ok = False
            try:
                if isinstance(res, str) and 'success=True' in res:
                    ok = True
            except Exception:
                ok = False

            if svc == 'activate' and ok:
                self._set_activation_led('on')
                self.log(f"SERVICE > activate succeeded: {res}")
            elif svc == 'deactivate' and ok:
                self._set_activation_led('off')
                self.log(f"SERVICE > deactivate succeeded: {res}")
            else:
                # call failed or unknown response
                self._set_activation_led('unknown')
                self.log(f"SERVICE > {svc}: {res}")
            return

        # general service messages
        self.log(f"SERVICE {svc} > {res}")

    def on_autoscroll_toggled(self, state):
        self.auto_scroll = bool(state == QtCore.Qt.Checked)
        self.log(f"Auto-scroll set to {self.auto_scroll}")

    def log(self, msg):
        ts = time.strftime('%H:%M:%S')

        # Default color
        color = "#00ff00"  # green for general messages

        if msg.startswith("CONN >"):
            color = "orange"
        elif msg.startswith("STATUS >"):
            color = "red"
        elif msg.startswith("SERVICE") or msg.startswith("Enqueued") or msg.startswith("Auto-scroll"):
            color = "cyan"

        # Append HTML formatted text
        self.txt_log.append(f'<span style="color:{color}">[{ts}] {msg}</span>')

        # Auto-scroll only if enabled
        if getattr(self, 'auto_scroll', False):
            self.txt_log.verticalScrollBar().setValue(self.txt_log.verticalScrollBar().maximum())

    def closeEvent(self, event):
        reply = QMessageBox.question(self,'Quit','Quit and shutdown GUI and ROS client?',
                                     QMessageBox.Yes|QMessageBox.No,QMessageBox.Yes)
        if reply == QMessageBox.Yes:
            try: self.ros_worker.stop(); time.sleep(0.2)
            except Exception: pass
            event.accept()
        else: event.ignore()

def main():
    QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
