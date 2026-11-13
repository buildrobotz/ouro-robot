"""Computer <-> dongle <-> robot. The only file that knows about bytes.

    link = OuroLink('/dev/cu.usbserial-0001')     # Windows: 'COM5', Linux: '/dev/ttyUSB0'
    link.send(angles_rad)                         # 8 joint targets, radians, sim convention
    tel = link.latest()                           # dict with acc/gyro (SI units), amps, latch, ...
"""
import json, struct, threading, time, pathlib
import numpy as np
import serial

ROBOT_ID = 7
CMD = struct.Struct('<BBBB8H')              # must match CmdPacket in firmware/*/ouro_common.h (20 bytes)
TEL = struct.Struct('<BBBBI6h6hhHBB')       # must match TelPacket in firmware/*/ouro_common.h (38 bytes)
# TelPacket.status bits (ouro_common.h ST_*): 1 live, 2 low battery, 4 command timeout, 8 servo rail under 4.5 V,
# 16 mid IMU fault, 32 head IMU fault, 64 INA219 fault (the faulty field then repeats its last good value).
# servo_mA covers +-6.4 A since the second 0.1 ohm shunt (docs/electronics.md). The policy still sees
# min(amps, 3.2) / 3.0 exactly as in the simulator: that clip lives in run_policy.py, not here.
ACC = 9.81 * 4 / 32768                      # +-4 g
GYR = np.deg2rad(1000) / 32768              # +-1000 dps
US_PER_RAD = 2000 / np.pi                   # 500..2500 us over 180 deg
CAL_FILE = pathlib.Path(__file__).with_name('calibration.json')


class OuroLink:
    def __init__(self, port, baud=921600):
        self.ser = serial.Serial(port, baud, timeout=0.01)
        cal = json.loads(CAL_FILE.read_text()) if CAL_FILE.exists() else {}
        self.centre_us = np.array(cal.get('centre_us', [1500] * 8), float)   # pulse where the joint is straight
        self.sign = np.array(cal.get('sign', [1] * 8), float)                # +1 / -1: which way is "curl up"
        self.seq, self._tel, self._lock = 0, None, threading.Lock()
        self.sent_at, self.rtt_ms = {}, float('nan')
        threading.Thread(target=self._reader, daemon=True).start()

    def send(self, angles_rad, enable=True):
        angles_rad = np.clip(np.asarray(angles_rad, float), -1.20, 1.50)    # same limits as sim/ouro_model.py: the printed end stop is at +90 deg, electronics touch at +92
        us = np.clip(self.centre_us + self.sign * np.asarray(angles_rad) * US_PER_RAD, 500, 2500).astype(int)
        self.send_us(us, enable)

    def send_us(self, us, enable=True):
        us = np.clip(np.asarray(us, float), 500, 2500)
        self.seq = (self.seq + 1) & 0xFF
        p = CMD.pack(0xC5, ROBOT_ID, self.seq, int(enable), *[int(u) for u in us])
        x = 0
        for b in p:
            x ^= b
        self.sent_at[self.seq] = time.perf_counter()
        self.ser.write(bytes([0xAA, 0x55, len(p)]) + p + bytes([x]))

    def limp(self):
        for _ in range(5):
            self.send_us([1500] * 8, enable=False); time.sleep(0.02)

    def latest(self):
        with self._lock:
            return self._tel

    def _reader(self):
        buf = bytearray()
        while True:
            buf += self.ser.read(256)
            while len(buf) >= 4:
                if buf[0] != 0xAA or buf[1] != 0x55:
                    del buf[0]; continue
                n = buf[2]
                if len(buf) < n + 4:
                    break
                p, chk = bytes(buf[3:3 + n]), buf[3 + n]
                del buf[:n + 4]
                x = 0
                for b in p:
                    x ^= b
                if x != chk or n != TEL.size:
                    continue
                v = TEL.unpack(p)
                t0 = self.sent_at.pop(v[3], None)  # first echo only: later frames repeat the seq 10 ms later
                if t0:
                    self.rtt_ms = 0.9 * self.rtt_ms + 0.1 * (time.perf_counter() - t0) * 1e3 if self.rtt_ms == self.rtt_ms else (time.perf_counter() - t0) * 1e3
                tel = dict(seq=v[2], t_ms=v[4],
                           acc_mid=np.array(v[5:8]) * ACC, gyro_mid=np.array(v[8:11]) * GYR,
                           acc_head=np.array(v[11:14]) * ACC, gyro_head=np.array(v[14:17]) * GYR,
                           amps=v[17] / 1000.0, batt_v=v[18] / 1000.0, latch=bool(v[19]), status=v[20],
                           rx_time=time.perf_counter())
                with self._lock:
                    self._tel = tel
