"""
平衡车 PID 智能调参上位机
依赖: pip install pyserial matplotlib requests
"""
import tkinter as tk
from tkinter import ttk, scrolledtext
import threading
import queue
import time
import json
import os
import re
import socket
import statistics
import serial
import serial.tools.list_ports
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque
from datetime import datetime
from agent import TuningMemory, run_agent_cycle, run_reflector, run_generate_report, AI_SECS

# ── 日志文件 ──────────────────────────────────────────────────
_LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
os.makedirs(_LOG_DIR, exist_ok=True)
_LOG_FILE = os.path.join(_LOG_DIR, datetime.now().strftime("%Y%m%d_%H%M%S") + ".log")
_log_lock = threading.Lock()

def file_log(msg: str):
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    line = f"[{ts}] {msg}\n"
    with _log_lock:
        try:
            with open(_LOG_FILE, "a", encoding="utf-8") as f:
                f.write(line)
        except Exception:
            pass

# ── 配置持久化 ────────────────────────────────────────────────
CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")

def load_config() -> dict:
    try:
        with open(CONFIG_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}

def save_config(data: dict):
    # api_key 明文存储在本地，注意不要将 config.json 上传到公开仓库
    try:
        with open(CONFIG_FILE, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
    except Exception:
        pass

# ── 中文字体 ──────────────────────────────────────────────────
def _setup_font():
    import matplotlib.font_manager as fm
    available = {f.name for f in fm.fontManager.ttflist}
    for name in ["Microsoft YaHei", "SimHei", "SimSun"]:
        if name in available:
            plt.rcParams["font.family"] = name
            break
    plt.rcParams["axes.unicode_minus"] = False
_setup_font()

BAUD          = 115200
WINDOW_SECS   = 10
SAMPLE_HZ     = 50
DEEPSEEK_URL  = "https://api.deepseek.com/chat/completions"
DEEPSEEK_MODEL = "deepseek-v4-flash"
MAX_PTS       = WINDOW_SECS * SAMPLE_HZ

# ── DataStore ─────────────────────────────────────────────────
class DataStore:
    def __init__(self):
        self.t       = deque(maxlen=MAX_PTS)
        self.pitch   = deque(maxlen=MAX_PTS)
        self.speed_l = deque(maxlen=MAX_PTS)
        self.speed_r = deque(maxlen=MAX_PTS)
        self.pwm     = deque(maxlen=MAX_PTS)
        self.kp = 30.0; self.ki = 0.0; self.kd = 0.8
        self.spd_kp = 0.5; self.spd_ki = 0.05; self.spd_kd = 0.0
        self.mode = 0
        self.pos_m = 0.0
        self.t0 = time.time()

    def push(self, pitch, sl, sr, pwm, kp, ki, kd, mode, spd_kp=None, spd_ki=None, spd_kd=None, pos_m=None):
        now = time.time() - self.t0
        self.t.append(now); self.pitch.append(pitch)
        self.speed_l.append(sl); self.speed_r.append(sr)
        self.pwm.append(pwm)
        self.kp = kp; self.ki = ki; self.kd = kd
        self.mode = int(mode)
        if spd_kp is not None: self.spd_kp = spd_kp
        if spd_ki is not None: self.spd_ki = spd_ki
        if spd_kd is not None: self.spd_kd = spd_kd
        if pos_m  is not None: self.pos_m  = pos_m

    def recent_pitch(self, secs=AI_SECS):
        if not self.t: return []
        cutoff = self.t[-1] - secs
        return [p for tv, p in zip(self.t, self.pitch) if tv >= cutoff]

# ── SerialReader ──────────────────────────────────────────────
class SerialReader(threading.Thread):
    MAX_RETRY = 3

    def __init__(self, port, data_q, log_q, status_q):
        super().__init__(daemon=True)
        self.port = port; self.data_q = data_q; self.log_q = log_q
        self.status_q = status_q
        self.ser = None; self.running = True

    def _open(self):
        if self.ser and self.ser.is_open:
            try: self.ser.close()
            except Exception: pass
        self.ser = serial.Serial(self.port, BAUD, timeout=0.1)

    def run(self):
        try:
            self._open()
            self.log_q.put(f"[串口] 已连接 {self.port}")
        except Exception as e:
            self.log_q.put(f"[错误] 串口打开失败: {e}")
            self.status_q.put("disconnected"); return

        retry = 0
        while self.running:
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if not line: continue
                retry = 0  # 成功收到数据，重置重试计数
                if line.startswith("DATA,"):
                    parts = line.split(",")
                    if len(parts) >= 10:
                        try:
                            self.data_q.put([float(x) for x in parts[1:13]])
                        except ValueError:
                            pass
                else:
                    self.log_q.put(f"[ESP32] {line}")
            except serial.SerialException as e:
                if not self.running: break
                retry += 1
                self.log_q.put(f"[串口] 连接中断，重连 {retry}/{self.MAX_RETRY}... ({e})")
                if retry >= self.MAX_RETRY:
                    self.log_q.put("[串口] 重连失败，请手动重新连接")
                    break
                time.sleep(1)
                try:
                    self._open()
                    self.log_q.put(f"[串口] 重连成功 {self.port}")
                    retry = 0
                except Exception as e2:
                    self.log_q.put(f"[串口] 重连失败: {e2}")
            except Exception as e:
                if not self.running: break
                self.log_q.put(f"[串口错误] {e}")

        self.status_q.put("disconnected")

    def send(self, cmd):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((cmd + "\n").encode())
            except Exception as e:
                self.log_q.put(f"[串口] 发送失败: {e}")

    def stop(self):
        self.running = False
        if self.ser:
            try: self.ser.close()
            except Exception: pass

# ── NetworkReader ──────────────────────────────────────────────
class NetworkReader(threading.Thread):
    MAX_RETRY = 3

    def __init__(self, host, port, data_q, log_q, status_q):
        super().__init__(daemon=True)
        self.host = host; self.port = port
        self.data_q = data_q; self.log_q = log_q; self.status_q = status_q
        self.sock = None; self.running = True

    def _open(self):
        if self.sock:
            try: self.sock.close()
            except Exception: pass
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(5)
        s.connect((self.host, self.port))
        s.settimeout(0.5)
        self.sock = s

    def run(self):
        try:
            self._open()
            self.log_q.put(f"[WiFi] 已连接 {self.host}:{self.port}")
        except Exception as e:
            self.log_q.put(f"[错误] TCP连接失败: {e}")
            self.status_q.put("disconnected"); return

        retry = 0
        buf = ""
        while self.running:
            try:
                chunk = self.sock.recv(256).decode("utf-8", errors="ignore")
                if not chunk:
                    raise ConnectionResetError("连接断开")
                retry = 0
                buf += chunk
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if not line: continue
                    if line.startswith("DATA,"):
                        parts = line.split(",")
                        if len(parts) >= 10:
                            try:
                                self.data_q.put([float(x) for x in parts[1:14]])
                            except ValueError:
                                pass
                    else:
                        self.log_q.put(f"[ESP32] {line}")
            except socket.timeout:
                continue
            except Exception as e:
                if not self.running: break
                retry += 1
                self.log_q.put(f"[WiFi] 断开，重连 {retry}/{self.MAX_RETRY}... ({e})")
                if retry >= self.MAX_RETRY:
                    self.log_q.put("[WiFi] 重连失败，请手动重新连接")
                    break
                time.sleep(1)
                try:
                    self._open()
                    self.log_q.put(f"[WiFi] 重连成功 {self.host}:{self.port}")
                    retry = 0; buf = ""
                except Exception as e2:
                    self.log_q.put(f"[WiFi] 重连失败: {e2}")

        self.status_q.put("disconnected")

    def send(self, cmd):
        if self.sock:
            try:
                self.sock.sendall((cmd + "\n").encode())
            except Exception as e:
                self.log_q.put(f"[WiFi] 发送失败: {e}")

    def stop(self):
        self.running = False
        if self.sock:
            try: self.sock.close()
            except Exception: pass

# ── App ───────────────────────────────────────────────────────
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("平衡车 PID 智能调参")
        self.geometry("1200x750")
        self.resizable(True, True)
        self.store  = DataStore()
        self.memory = TuningMemory()
        self.data_q = queue.Queue()
        self.log_q  = queue.Queue()
        self.ai_q   = queue.Queue()
        self.status_q = queue.Queue()
        self.reader = None
        self._ai_angle_busy = False
        self._ai_speed_busy = False
        self._pending_restore = False   # 等 ESP32 ready 信号再下发参数
        # 上次调参前的基准std（用于回滚判断）
        self._ai_prev_params = None   # (kp, ki, kd) 调参前
        self._ai_prev_std    = None   # 调参前的振荡std
        self._ai_verify_tag  = None   # 正在验证的tag
        self._ai_verify_ts   = 0      # 开始验证的时间
        self._auto_last = 0
        self._build_ui()
        # 加载持久化配置
        cfg = load_config()
        self.api_key_var.set(cfg.get("api_key", ""))
        self.api_url_var.set(cfg.get("api_url", DEEPSEEK_URL))
        self.ip_var.set(cfg.get("tcp_ip", ""))
        self.tcpport_var.set(str(cfg.get("tcp_port", 8888)))
        self.kp_var.set(str(cfg.get("kp", "30.0")))
        self.ki_var.set(str(cfg.get("ki", "0.0")))
        self.kd_var.set(str(cfg.get("kd", "0.8")))
        self.spd_kp_var.set(str(cfg.get("spd_kp", "0.5")))
        self.spd_ki_var.set(str(cfg.get("spd_ki", "0.05")))
        self.spd_kd_var.set(str(cfg.get("spd_kd", "0.0")))
        self.pos_gain_var.set(str(cfg.get("pos_gain", "0.3")))
        self.diff_kp_var.set(str(cfg.get("diff_kp", "5.0")))
        self.offset_var.set(str(cfg.get("offset", "0.0")))
        self._refresh()

    # ── build UI ──────────────────────────────────────────────
    def _build_ui(self):
        # ── top bar ──
        top = tk.Frame(self, pady=4)
        top.pack(fill=tk.X, padx=8)
        tk.Label(top, text="串口:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(top, textvariable=self.port_var, width=10)
        self.port_cb.pack(side=tk.LEFT, padx=2)
        tk.Button(top, text="刷新", command=self._refresh_ports).pack(side=tk.LEFT)
        # WiFi IP / Port
        tk.Label(top, text="IP:").pack(side=tk.LEFT, padx=(8,2))
        self.ip_var = tk.StringVar(value="")
        tk.Entry(top, textvariable=self.ip_var, width=14).pack(side=tk.LEFT)
        tk.Label(top, text="端口:").pack(side=tk.LEFT, padx=(4,2))
        self.tcpport_var = tk.StringVar(value="8888")
        tk.Entry(top, textvariable=self.tcpport_var, width=6).pack(side=tk.LEFT)
        # 连接模式选择
        self.conn_mode = tk.StringVar(value="serial")
        tk.Radiobutton(top, text="串口", variable=self.conn_mode, value="serial").pack(side=tk.LEFT, padx=(6,0))
        tk.Radiobutton(top, text="WiFi", variable=self.conn_mode, value="wifi").pack(side=tk.LEFT)
        self.btn_conn = tk.Button(top, text="连接", command=self._toggle_connect,
                                  bg="#4CAF50", fg="white", width=6)
        self.btn_conn.pack(side=tk.LEFT, padx=6)
        tk.Label(top, text="DeepSeek Key:").pack(side=tk.LEFT, padx=(16,2))
        self.api_key_var = tk.StringVar()
        tk.Entry(top, textvariable=self.api_key_var, width=36, show="*").pack(side=tk.LEFT)
        tk.Label(top, text="API URL:").pack(side=tk.LEFT, padx=(8,2))
        self.api_url_var = tk.StringVar(value="https://api.deepseek.com/chat/completions")
        tk.Entry(top, textvariable=self.api_url_var, width=36).pack(side=tk.LEFT)
        # pitch live display
        self.pitch_live = tk.Label(top, text="Pitch: --.-°", font=("Consolas", 13, "bold"),
                                   fg="#1565C0", width=14)
        self.pitch_live.pack(side=tk.RIGHT, padx=12)
        self._refresh_ports()

        # ── main area ──
        main = tk.Frame(self)
        main.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        # plots
        pf = tk.Frame(main)
        pf.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.fig = Figure(figsize=(7, 5), dpi=96)
        self.ax_pitch = self.fig.add_subplot(311)
        self.ax_speed = self.fig.add_subplot(312)
        self.ax_pwm   = self.fig.add_subplot(313)
        # 预建 Line2D 对象，后续只更新数据，不重建坐标轴
        self._ln_pitch, = self.ax_pitch.plot([], [], 'b-', lw=0.8, label="pitch")
        self._ln_sl,    = self.ax_speed.plot([], [], 'g-', lw=0.8, label="Left")
        self._ln_sr,    = self.ax_speed.plot([], [], 'm-', lw=0.8, label="Right")
        self._ln_pwm,   = self.ax_pwm.plot([], [], 'r-', lw=0.8, label="PWM")
        self.ax_pitch.axhline(0, color='r', lw=0.5, ls='--')
        self.ax_pitch.set_ylabel("deg", fontsize=8); self.ax_pitch.set_title("Pitch (deg)", fontsize=9)
        self.ax_pitch.grid(True, alpha=0.3); self.ax_pitch.legend(fontsize=7)
        self.ax_speed.set_ylabel("m/s", fontsize=8); self.ax_speed.set_title("Speed (m/s)", fontsize=9)
        self.ax_speed.grid(True, alpha=0.3); self.ax_speed.legend(fontsize=7)
        self.ax_pwm.set_ylabel("PWM", fontsize=8); self.ax_pwm.set_title("PWM Output", fontsize=9)
        self.ax_pwm.grid(True, alpha=0.3); self.ax_pwm.legend(fontsize=7)
        self.fig.tight_layout(pad=1.2)
        self.canvas = FigureCanvasTkAgg(self.fig, master=pf)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # right panel
        right = tk.Frame(main, width=300)
        right.pack(side=tk.RIGHT, fill=tk.Y, padx=(8,0))
        right.pack_propagate(False)

        # ── 零点偏移（最顶部，最常用）──
        off_frame = tk.LabelFrame(right, text="Pitch 零点偏移", padx=6, pady=4)
        off_frame.pack(fill=tk.X, pady=3)
        tk.Label(off_frame, text="当前 Pitch 实时值见右上角\n车竖立时填入该值作为偏移",
                 fg="#555", font=("",8), justify=tk.LEFT).pack(anchor=tk.W)
        off_row = tk.Frame(off_frame)
        off_row.pack(fill=tk.X, pady=3)
        self.offset_var = tk.StringVar(value="0.0")
        self.offset_entry = tk.Entry(off_row, textvariable=self.offset_var, width=8,
                                     font=("Consolas",11))
        self.offset_entry.pack(side=tk.LEFT)
        tk.Button(off_row, text="设置偏移", bg="#7B1FA2", fg="white",
                  command=self._send_offset).pack(side=tk.LEFT, padx=4)
        tk.Button(off_row, text="用当前值", bg="#9C27B0", fg="white",
                  command=self._use_current_pitch).pack(side=tk.LEFT)

        # ── 角度环 PID ──
        ang_frame = tk.LabelFrame(right, text="角度环 PID", padx=6, pady=4)
        ang_frame.pack(fill=tk.X, pady=3)
        self.kp_var = tk.StringVar(value="30.0")
        self.ki_var = tk.StringVar(value="0.0")
        self.kd_var = tk.StringVar(value="0.8")
        for lbl, var in [("Kp", self.kp_var), ("Ki", self.ki_var), ("Kd", self.kd_var)]:
            r = tk.Frame(ang_frame); r.pack(fill=tk.X, pady=1)
            tk.Label(r, text=lbl, width=3).pack(side=tk.LEFT)
            tk.Entry(r, textvariable=var, width=10, font=("Consolas",10)).pack(side=tk.LEFT, padx=3)
        tk.Button(ang_frame, text="发送角度环参数", bg="#1565C0", fg="white",
                  command=self._send_angle_pid).pack(fill=tk.X, pady=3)
        self.btn_ai_agent = tk.Button(ang_frame, text="▶ 智能体调参 (Planner+Tuner+Reflector)",
                                      bg="#FF6F00", fg="white",
                                      command=self._run_agent)
        self.btn_ai_agent.pack(fill=tk.X)

        # ── 速度环 PID ──
        spd_frame = tk.LabelFrame(right, text="速度环 PID (角度环调好后再开)", padx=6, pady=4)
        spd_frame.pack(fill=tk.X, pady=3)
        self.spd_kp_var = tk.StringVar(value="0.5")
        self.spd_ki_var = tk.StringVar(value="0.05")
        self.spd_kd_var = tk.StringVar(value="0.0")
        for lbl, var in [("Kp", self.spd_kp_var), ("Ki", self.spd_ki_var), ("Kd", self.spd_kd_var)]:
            r = tk.Frame(spd_frame); r.pack(fill=tk.X, pady=1)
            tk.Label(r, text=lbl, width=3).pack(side=tk.LEFT)
            tk.Entry(r, textvariable=var, width=10, font=("Consolas",10)).pack(side=tk.LEFT, padx=3)
        tk.Button(spd_frame, text="发送速度环参数", bg="#1B5E20", fg="white",
                  command=self._send_speed_pid).pack(fill=tk.X, pady=3)

        # ── 位置保持 ──
        pos_frame = tk.LabelFrame(right, text="位置保持 (双环模式生效)", padx=6, pady=4)
        pos_frame.pack(fill=tk.X, pady=3)
        pr = tk.Frame(pos_frame); pr.pack(fill=tk.X, pady=1)
        tk.Label(pr, text="位置Kp", width=7).pack(side=tk.LEFT)
        self.pos_gain_var = tk.StringVar(value="0.3")
        tk.Entry(pr, textvariable=self.pos_gain_var, width=8, font=("Consolas",10)).pack(side=tk.LEFT, padx=3)
        tk.Button(pr, text="设置", bg="#00695C", fg="white",
                  command=self._send_pos_gain).pack(side=tk.LEFT)
        pb = tk.Frame(pos_frame); pb.pack(fill=tk.X, pady=2)
        self.pos_label = tk.Label(pb, text="位移: 0.000 m", font=("Consolas", 9), fg="#00695C")
        self.pos_label.pack(side=tk.LEFT)
        tk.Button(pb, text="归零位置", bg="#00838F", fg="white",
                  command=self._zero_pos).pack(side=tk.RIGHT)

        # ── 差速补偿 ──
        diff_frame = tk.LabelFrame(right, text="差速补偿", padx=6, pady=4)
        diff_frame.pack(fill=tk.X, pady=3)
        dr = tk.Frame(diff_frame); dr.pack(fill=tk.X, pady=1)
        tk.Label(dr, text="差速Kp", width=7).pack(side=tk.LEFT)
        self.diff_kp_var = tk.StringVar(value="5.0")
        tk.Entry(dr, textvariable=self.diff_kp_var, width=8, font=("Consolas",10)).pack(side=tk.LEFT, padx=3)
        tk.Button(dr, text="设置", bg="#4527A0", fg="white",
                  command=self._send_diff_kp).pack(side=tk.LEFT)
        mode_frame = tk.LabelFrame(right, text="控制模式", padx=6, pady=4)
        mode_frame.pack(fill=tk.X, pady=3)
        self.mode_label = tk.Label(mode_frame, text="● 角度环 PID 调试",
                                   fg="#E65100", font=("",9,"bold"))
        self.mode_label.pack(anchor=tk.W)
        mr = tk.Frame(mode_frame); mr.pack(fill=tk.X, pady=2)
        tk.Button(mr, text="模式1: 只调角度环", bg="#E65100", fg="white",
                  command=lambda: self._set_mode(0)).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0,2))
        tk.Button(mr, text="模式2: 完整双环", bg="#1B5E20", fg="white",
                  command=lambda: self._set_mode(1)).pack(side=tk.LEFT, fill=tk.X, expand=True)

        # ── 自动调参 ──
        auto_frame = tk.LabelFrame(right, text="自动调参", padx=6, pady=4)
        auto_frame.pack(fill=tk.X, pady=3)
        self.auto_angle_var = tk.BooleanVar(value=False)
        tk.Checkbutton(auto_frame, text="自动智能体调参 (每30s)",
                       variable=self.auto_angle_var).pack(anchor=tk.W)

        # ── IMU校准 ──
        tk.Button(right, text="IMU 校准 (小车放平静止)", bg="#37474F", fg="white",
                  command=lambda: self._send_raw("CALIB")).pack(fill=tk.X, pady=3)

        # ── 生成报告 ──
        tk.Button(right, text="生成调参报告 + 凝练经验", bg="#4527A0", fg="white",
                  command=self._run_report).pack(fill=tk.X, pady=3)

        # ── 日志 ──
        log_frame = tk.LabelFrame(right, text="日志", padx=4, pady=4)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=3)
        self.log_box = scrolledtext.ScrolledText(log_frame, font=("Consolas", 8))
        self.log_box.pack(fill=tk.BOTH, expand=True)

    # ── helpers ───────────────────────────────────────────────
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports: self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.reader and self.reader.is_alive():
            self.reader.stop()
            self.reader.join(timeout=2)
            self.reader = None
            self.btn_conn.config(text="连接", bg="#4CAF50")
        else:
            while not self.status_q.empty():
                try: self.status_q.get_nowait()
                except Exception: pass
            if self.conn_mode.get() == "wifi":
                host = self.ip_var.get().strip()
                if not host: self._log("请填写 ESP32 IP 地址"); return
                try: port = int(self.tcpport_var.get().strip())
                except ValueError: port = 8888
                # 保存IP到config
                cfg = load_config(); cfg.update({"tcp_ip": host, "tcp_port": port}); save_config(cfg)
                self.reader = NetworkReader(host, port, self.data_q, self.log_q, self.status_q)
            else:
                port = self.port_var.get()
                if not port: self._log("请选择串口"); return
                self.reader = SerialReader(port, self.data_q, self.log_q, self.status_q)
            self.reader.start()
            self.btn_conn.config(text="断开", bg="#f44336")
            self._pending_restore = True   # 等 ESP32 ready 再下发参数

    def _restore_params_to_esp32(self):
        """重连后把 config.json 里保存的参数下发到 ESP32"""
        if not self.reader or not self.reader.is_alive():
            return
        cfg = load_config()
        try:
            kp  = float(cfg.get("kp",  30.0))
            ki  = float(cfg.get("ki",   0.0))
            kd  = float(cfg.get("kd",   0.8))
            skp = float(cfg.get("spd_kp", 0.5))
            ski = float(cfg.get("spd_ki", 0.05))
            skd = float(cfg.get("spd_kd", 0.0))
            off = float(cfg.get("offset", 0.0))
        except (ValueError, TypeError):
            return
        # 同步更新 UI，保证显示和下发一致
        self.kp_var.set(f"{kp:.3f}"); self.ki_var.set(f"{ki:.4f}"); self.kd_var.set(f"{kd:.3f}")
        self.spd_kp_var.set(f"{skp:.3f}"); self.spd_ki_var.set(f"{ski:.4f}")
        self.offset_var.set(f"{off:.3f}")
        self._send_raw(f"SET,{kp:.4f},{ki:.4f},{kd:.4f}")
        self._send_raw(f"SETSPD,{skp:.4f},{ski:.4f},{skd:.4f}")
        self._send_raw(f"OFFSET,{off:.3f}")
        try:
            pg = float(cfg.get("pos_gain", 0.3))
            self._send_raw(f"SETPOS,{pg:.4f}")
        except (ValueError, TypeError):
            pass
        try:
            dg = float(cfg.get("diff_kp", 5.0))
            self._send_raw(f"SETDIFF,{dg:.4f}")
        except (ValueError, TypeError):
            pass
        self._log(f"[恢复] 已下发保存参数 角度Kp={kp} Ki={ki} Kd={kd} | 速度Kp={skp} Ki={ski} | Offset={off}")

    def _send_raw(self, cmd):
        if self.reader: self.reader.send(cmd)
        else: self._log("未连接串口")

    def _log(self, msg):
        ts = time.strftime("%H:%M:%S")
        line = f"[{ts}] {msg}"
        self.log_box.insert(tk.END, line + "\n")
        self.log_box.see(tk.END)
        file_log(msg)

    def _send_offset(self):
        try:
            val = float(self.offset_var.get())
        except ValueError:
            self._log("[偏移] 格式错误，请输入数字"); return
        self._send_raw(f"OFFSET,{val:.3f}")
        self._log(f"[偏移] 已发送 OFFSET={val:.3f}°")
        self._save_pid_config()

    def _use_current_pitch(self):
        if not self.store.pitch:
            self._log("[偏移] 暂无数据"); return
        val = self.store.pitch[-1]
        self.offset_var.set(f"{val:.2f}")
        self._send_raw(f"OFFSET,{val:.3f}")
        self._log(f"[偏移] 用当前 pitch={val:.2f}° 作为零点")
        self._save_pid_config()

    def _send_angle_pid(self):
        try:
            kp = float(self.kp_var.get())
            ki = float(self.ki_var.get())
            kd = float(self.kd_var.get())
        except ValueError:
            self._log("[角度环] 参数格式错误"); return
        self._send_raw(f"SET,{kp:.4f},{ki:.4f},{kd:.4f}")
        self._log(f"[角度环] 发送 Kp={kp} Ki={ki} Kd={kd}")
        self._save_pid_config()

    def _send_speed_pid(self):
        try:
            kp = float(self.spd_kp_var.get())
            ki = float(self.spd_ki_var.get())
            kd = float(self.spd_kd_var.get())
        except ValueError:
            self._log("[速度环] 参数格式错误"); return
        self._send_raw(f"SETSPD,{kp:.4f},{ki:.4f},{kd:.4f}")
        self._log(f"[速度环] 发送 Kp={kp} Ki={ki} Kd={kd}")
        self._save_pid_config()

    def _send_pos_gain(self):
        try:
            pg = float(self.pos_gain_var.get())
        except ValueError:
            self._log("[位置] 位置Kp格式错误"); return
        self._send_raw(f"SETPOS,{pg:.4f}")
        self._log(f"[位置] 发送 POS_GAIN={pg:.4f}")
        self._save_pid_config()

    def _zero_pos(self):
        self._send_raw("ZEROPOS")
        self._log("[位置] 归零位置")

    def _send_diff_kp(self):
        try:
            dg = float(self.diff_kp_var.get())
        except ValueError:
            self._log("[差速] Kp格式错误"); return
        self._send_raw(f"SETDIFF,{dg:.4f}")
        self._log(f"[差速] 发送 DIFF_KP={dg:.4f}")
        self._save_pid_config()

    def _save_pid_config(self):
        """把当前 UI 上的所有参数保存到 config.json"""
        cfg = load_config()
        try:
            cfg.update({
                "kp":       float(self.kp_var.get()),
                "ki":       float(self.ki_var.get()),
                "kd":       float(self.kd_var.get()),
                "spd_kp":   float(self.spd_kp_var.get()),
                "spd_ki":   float(self.spd_ki_var.get()),
                "spd_kd":   float(self.spd_kd_var.get()),
                "pos_gain": float(self.pos_gain_var.get()),
                "diff_kp":  float(self.diff_kp_var.get()),
                "offset":   float(self.offset_var.get()),
            })
            save_config(cfg)
        except ValueError:
            pass

    def _set_mode(self, m):
        self._send_raw(f"MODE,{m}")
        self._log(f"[模式] 切换到 {'角度环PD调试' if m==0 else '完整双环'}")

    def _show_report(self, text):
        win = tk.Toplevel(self)
        win.title("调参报告")
        win.geometry("700x600")
        st = scrolledtext.ScrolledText(win, font=("Consolas", 9), wrap=tk.WORD)
        st.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        st.insert(tk.END, text)
        st.config(state=tk.DISABLED)
        tk.Button(win, text="关闭", command=win.destroy).pack(pady=4)

    def _run_report(self):
        key = self.api_key_var.get().strip()
        url = self.api_url_var.get().strip()
        if not key: self._log("[报告] 请填写 API Key"); return
        self._log("[报告] 生成中，请稍候...")
        threading.Thread(target=run_generate_report,
                         args=(key, url, self.store, self.memory, self.log_q, self.ai_q),
                         daemon=True).start()

    def _run_agent(self):
        key = self.api_key_var.get().strip()
        url = self.api_url_var.get().strip()
        if not key: self._log("[Agent] 请填写 API Key"); return
        if self._ai_angle_busy: return
        cfg = load_config(); cfg.update({"api_key": key, "api_url": url}); save_config(cfg)
        self._ai_angle_busy = True
        self.btn_ai_agent.config(state=tk.DISABLED, text="智能体运行中...")
        threading.Thread(target=run_agent_cycle,
                         args=(key, url, self.store, self.memory, self.log_q, self.ai_q),
                         daemon=True).start()

    # ── refresh loop ──────────────────────────────────────────
    def _refresh(self):
        # 检查串口断线通知
        while not self.status_q.empty():
            status = self.status_q.get_nowait()
            if status == "disconnected":
                self.reader = None
                self.btn_conn.config(text="连接", bg="#4CAF50")
                self._log("[串口] 已断开，请重新连接")

        while not self.log_q.empty():
            msg = self.log_q.get_nowait()
            self._log(msg)
            if self._pending_restore:
                # 串口/WiFi 连接成功后 ESP32 已 ready（WiFi模式主动连）→ 立即恢复
                if "已连接" in msg and ("串口" in msg or "WiFi" in msg):
                    self._pending_restore = False
                    self.after(500, self._restore_params_to_esp32)
                # 串口模式：等 ESP32 启动完成信号再恢复（避免 ESP32 还没初始化完就下发）
                elif "Balance car ready" in msg:
                    self._pending_restore = False
                    self.after(200, self._restore_params_to_esp32)

        updated = False
        _data_log_interval = 1.0  # 每秒写一次 DATA 到日志
        while not self.data_q.empty():
            vals = self.data_q.get_nowait()
            if len(vals) >= 9:
                pitch, sl, sr, pwm_l, pwm_r, kp, ki, kd, mode = vals[:9]
                spd_kp = vals[9]  if len(vals) > 9  else None
                spd_ki = vals[10] if len(vals) > 10 else None
                spd_kd = vals[11] if len(vals) > 11 else None
                pos_m  = vals[12] if len(vals) > 12 else None
                self.store.push(pitch, sl, sr, (pwm_l+pwm_r)/2, kp, ki, kd, mode, spd_kp, spd_ki, spd_kd, pos_m)
                # 每秒写一次传感器数据到日志文件
                now = time.time()
                if not hasattr(self, '_last_data_log') or now - self._last_data_log >= _data_log_interval:
                    self._last_data_log = now
                    file_log(f"DATA pitch={pitch:+.2f}° sl={sl:.3f} sr={sr:.3f} pwm={((pwm_l+pwm_r)/2):.0f} Kp={kp} Ki={ki} Kd={kd} mode={int(mode)}")
                # live pitch display
                color = "#C62828" if abs(pitch) > 20 else "#1565C0"
                self.pitch_live.config(text=f"Pitch: {pitch:+.1f}°", fg=color)
                # mode label
                if int(mode) == 0:
                    self.mode_label.config(text="● 角度环 PID 调试", fg="#E65100")
                else:
                    self.mode_label.config(text="● 完整双环运行", fg="#1B5E20")
                # pos_m display
                self.pos_label.config(text=f"位移: {self.store.pos_m:+.3f} m")
                updated = True

        if updated:
            self._update_plots()

        while not self.ai_q.empty():
            tag, data = self.ai_q.get_nowait()
            if tag == "agent":
                self._ai_angle_busy = False
                self._ai_speed_busy = False
                try:
                    self.btn_ai_agent.config(state=tk.NORMAL, text="▶ 智能体调参 (Planner+Tuner+Reflector)")
                except Exception:
                    pass
                if not data or data.get("action") == "observe":
                    continue
                action = data.get("action", "tune_angle")
                pitches = self.store.recent_pitch(AI_SECS)
                self._ai_prev_std = statistics.stdev(pitches) if len(pitches) > 1 else 999
                self._ai_verify_tag = action
                self._ai_verify_ts = time.time()
                self._ai_verify_reason = data.get("reason", "")
                if action == "tune_angle":
                    kp = data.get("kp", self.store.kp)
                    ki = data.get("ki", self.store.ki)
                    kd = data.get("kd", self.store.kd)
                    self._ai_prev_params = (self.store.kp, self.store.ki, self.store.kd)
                    self.kp_var.set(f"{kp:.3f}"); self.ki_var.set(f"{ki:.4f}"); self.kd_var.set(f"{kd:.3f}")
                    self._send_raw(f"SET,{kp:.4f},{ki:.4f},{kd:.4f}")
                    self._log(f"[Tuner] 下发角度环 Kp={kp:.3f} Ki={ki:.4f} Kd={kd:.3f} | {AI_SECS}s后验证")
                    self._save_pid_config()
                else:
                    skp = data.get("spd_kp", self.store.spd_kp)
                    ski = data.get("spd_ki", self.store.spd_ki)
                    skd = data.get("spd_kd", self.store.spd_kd)
                    self._ai_prev_params = (self.store.spd_kp, self.store.spd_ki, self.store.spd_kd)
                    self.spd_kp_var.set(f"{skp:.3f}"); self.spd_ki_var.set(f"{ski:.4f}")
                    self._send_raw(f"SETSPD,{skp:.4f},{ski:.4f},{skd:.4f}")
                    self._log(f"[Tuner] 下发速度环 Kp={skp:.3f} Ki={ski:.4f} | {AI_SECS}s后验证")
                    self._save_pid_config()
            # 兼容旧tag
            elif tag in ("angle", "speed"):
                self._ai_angle_busy = False
                self._ai_speed_busy = False
            elif tag == "report":
                if data:
                    self._show_report(data)

        # AI效果验证 + Reflector
        if self._ai_verify_tag and time.time() - self._ai_verify_ts >= AI_SECS:
            pitches = self.store.recent_pitch(AI_SECS)
            if len(pitches) > 1 and self._ai_prev_std is not None:
                new_std = statistics.stdev(pitches)
                prev_std = self._ai_prev_std
                is_angle = self._ai_verify_tag == "tune_angle"
                p = self._ai_prev_params
                outcome = "better" if new_std < prev_std * 0.9 else ("worse" if new_std > prev_std * 1.2 else "neutral")

                if outcome == "worse":
                    if is_angle:
                        self.kp_var.set(f"{p[0]:.3f}"); self.ki_var.set(f"{p[1]:.4f}"); self.kd_var.set(f"{p[2]:.3f}")
                        self._send_raw(f"SET,{p[0]:.4f},{p[1]:.4f},{p[2]:.4f}")
                        pa = {"kp": p[0], "ki": p[1], "kd": p[2]}
                    else:
                        self.spd_kp_var.set(f"{p[0]:.3f}"); self.spd_ki_var.set(f"{p[1]:.4f}")
                        self._send_raw(f"SETSPD,{p[0]:.4f},{p[1]:.4f},{p[2]:.4f}")
                        pa = {"kp": p[0], "ki": p[1], "kd": p[2]}
                    self._log(f"[验证] 效果变差(std {prev_std:.2f}→{new_std:.2f})，已回滚")
                    self._save_pid_config()
                else:
                    pa = {"kp": self.store.kp, "ki": self.store.ki, "kd": self.store.kd}
                    self._log(f"[验证] {'改善' if outcome=='better' else '持平'}(std {prev_std:.2f}→{new_std:.2f})，保留")

                # 触发 Reflector 写经验
                attempt = {
                    "ts": datetime.now().isoformat(),
                    "loop": "角度" if is_angle else "速度",
                    "params_before": {"kp": p[0], "ki": p[1], "kd": p[2]},
                    "params_after": pa,
                    "std_before": prev_std, "std_after": new_std,
                    "mean_before": statistics.mean(pitches) if pitches else 0,
                    "mean_after": statistics.mean(self.store.recent_pitch(AI_SECS)) if self.store.recent_pitch(AI_SECS) else 0,
                    "outcome": outcome,
                    "reason": getattr(self, "_ai_verify_reason", ""),
                }
                self.memory.add_attempt(attempt)
                key = self.api_key_var.get().strip()
                url = self.api_url_var.get().strip()
                if key:
                    threading.Thread(target=run_reflector,
                                     args=(key, url, attempt, self.memory, self.log_q),
                                     daemon=True).start()

            self._ai_verify_tag = None; self._ai_prev_params = None; self._ai_prev_std = None

        # auto agent
        now = time.time()
        if now - self._auto_last >= 30:
            if self.auto_angle_var.get() and not self._ai_angle_busy:
                self._auto_last = now; self._run_agent()

        self.after(50, self._refresh)

    def _update_plots(self):
        t = list(self.store.t)
        if not t: return
        t_end = t[-1]
        t0 = t_end - WINDOW_SECS

        # 转成相对时间（0 = 最新，-10 = 10秒前），X轴始终是 -10~0
        # 统一用同一次 zip 过滤，保证各列表长度一致
        pitch_s = list(self.store.pitch)
        sl_s    = list(self.store.speed_l)
        sr_s    = list(self.store.speed_r)
        pwm_s   = list(self.store.pwm)
        n = min(len(t), len(pitch_s), len(sl_s), len(sr_s), len(pwm_s))
        rows = [(tv - t_end, p, sl, sr, pw)
                for tv, p, sl, sr, pw in zip(t[-n:], pitch_s[-n:], sl_s[-n:], sr_s[-n:], pwm_s[-n:])
                if tv >= t0]
        if not rows:
            return
        tt, pitch, sl, sr, pwm = zip(*rows)

        self._ln_pitch.set_data(tt, pitch)
        self._ln_sl.set_data(tt, sl)
        self._ln_sr.set_data(tt, sr)
        self._ln_pwm.set_data(tt, pwm)

        self.ax_pitch.set_xlim(-WINDOW_SECS, 0)
        self.ax_speed.set_xlim(-WINDOW_SECS, 0)
        self.ax_pwm.set_xlim(-WINDOW_SECS, 0)

        for ax, data in [(self.ax_pitch, pitch), (self.ax_speed, sl+sr), (self.ax_pwm, pwm)]:
            if data:
                mn, mx = min(data), max(data)
                pad = max((mx - mn) * 0.1, 0.5)
                ax.set_ylim(mn - pad, mx + pad)

        self.canvas.draw_idle()

if __name__ == "__main__":
    App().mainloop()

