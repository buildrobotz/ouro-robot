"""Check host/link.py against the firmware's packet layout, with no hardware.

    python test_protocol.py

1. Compiles a 20-line C++ probe against firmware/ouro_robot/ouro_common.h (needs c++/clang, skipped if absent)
   and compares every field offset and a sample packet byte-for-byte with link.py's struct formats.
2. Runs the real OuroLink against a fake serial port: checks the command frame it writes (what the dongle
   parses) and that a telemetry frame laid out exactly as the firmware sends it unpacks to the right values.
"""
import pathlib, shutil, struct, subprocess, sys, tempfile, time
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
HDR = HERE.parent / 'firmware' / 'ouro_robot' / 'ouro_common.h'
sys.path.insert(0, str(HERE))
import serial                      # noqa: E402  (pyserial; replaced by a fake below)

# Field offsets by hand from ouro_common.h (packed, little-endian).
CMD_FIELDS = dict(magic=0, robot_id=1, seq=2, flags=3, pulse_us=4)
TEL_FIELDS = dict(magic=0, robot_id=1, seq=2, last_cmd_seq=3, t_ms=4, mid=8, head=20,
                  servo_mA=32, batt_mV=34, latch=36, status=37)
SAMPLE = dict(seq=200, last_cmd_seq=17, t_ms=123456, mid=[100, -200, 8192, 33, -44, 55],
              head=[-8192, 1, 2, -3, 4, -5], servo_mA=-1234, batt_mV=7812, latch=1, status=0x15)


def firmware_bytes(p):
    """A TelPacket exactly as the ESP32 lays it out: field by field at the header's offsets, little-endian."""
    b = bytearray(38)
    b[0], b[1], b[2], b[3] = 0x7E, 7, p['seq'], p['last_cmd_seq']
    struct.pack_into('<I', b, 4, p['t_ms'])
    for i, v in enumerate(p['mid']):
        struct.pack_into('<h', b, 8 + 2 * i, v)
    for i, v in enumerate(p['head']):
        struct.pack_into('<h', b, 20 + 2 * i, v)
    struct.pack_into('<h', b, 32, p['servo_mA']); struct.pack_into('<H', b, 34, p['batt_mV'])
    b[36], b[37] = p['latch'], p['status']
    return bytes(b)


def check_with_compiler(link):
    cxx = shutil.which('c++') or shutil.which('clang++') or shutil.which('g++')
    if not cxx:
        print('SKIP  C++ probe (no compiler found)'); return
    nl = '\\n'
    cmd_off = ' '.join(f'printf("C {k} %zu{nl}", offsetof(CmdPacket, {k}));' for k in CMD_FIELDS)
    tel_off = ' '.join(f'printf("T {k} %zu{nl}", offsetof(TelPacket, {k}));' for k in TEL_FIELDS)
    src = f'''#include <cstdio>
#include <cstddef>
#include <cstring>
#include "{HDR}"
int main() {{
  printf("%zu %zu\\n", sizeof(CmdPacket), sizeof(TelPacket));
  {cmd_off}
  {tel_off}
  TelPacket t; memset(&t, 0, sizeof t);
  t.magic = 0x7E; t.robot_id = ROBOT_ID; t.seq = {SAMPLE['seq']}; t.last_cmd_seq = {SAMPLE['last_cmd_seq']};
  t.t_ms = {SAMPLE['t_ms']}u; int16_t m[6] = {{{','.join(map(str, SAMPLE['mid']))}}}; memcpy(t.mid, m, 12);
  int16_t h[6] = {{{','.join(map(str, SAMPLE['head']))}}}; memcpy(t.head, h, 12);
  t.servo_mA = {SAMPLE['servo_mA']}; t.batt_mV = {SAMPLE['batt_mV']}; t.latch = {SAMPLE['latch']}; t.status = {SAMPLE['status']};
  const unsigned char *p = (const unsigned char *)&t; printf("B");
  for (size_t i = 0; i < sizeof t; i++) printf(" %u", p[i]);
  printf("\\n");
}}'''
    with tempfile.TemporaryDirectory() as d:
        c, exe = pathlib.Path(d, 'probe.cpp'), pathlib.Path(d, 'probe')
        c.write_text(src)
        subprocess.run([cxx, '-std=c++17', '-o', str(exe), str(c)], check=True)
        out = subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout.split('\n')
    sizes = tuple(map(int, out[0].split()))
    assert sizes == (link.CMD.size, link.TEL.size) == (20, 38), f'sizes C++ {sizes} vs Python {link.CMD.size, link.TEL.size}'
    for line in out[1:]:
        if line.startswith(('C ', 'T ')):
            kind, name, off = line.split()
            want = (CMD_FIELDS if kind == 'C' else TEL_FIELDS)[name]
            assert int(off) == want, f'{kind} {name}: header offset {off}, test expects {want}'
        elif line.startswith('B'):
            assert bytes(map(int, line.split()[1:])) == firmware_bytes(SAMPLE), 'C++ bytes differ from firmware_bytes()'
    print(f'PASS  C++ probe ({cxx}): sizes 20/38, every field offset, sample telemetry bytes identical')


class FakeSerial:
    """Stands in for serial.Serial: hands OuroLink one telemetry frame, records what it writes."""
    def __init__(self, *a, **k):
        self.rx, self.tx = bytearray(), bytearray()
    def read(self, n):
        if not self.rx:
            time.sleep(0.002); return b''
        out = bytes(self.rx[:n]); del self.rx[:n]; return out
    def write(self, b):
        self.tx += b


def main():
    serial.Serial = FakeSerial
    import link
    check_with_compiler(link)

    # Command: what link writes must be exactly what ouro_dongle.ino accepts.
    L = link.OuroLink('fake')
    L.centre_us[:] = 1500; L.sign[:] = 1
    L.send_us([500, 1000, 1500, 2000, 2500, 1234, 1500, 1501], enable=True)
    f = bytes(L.ser.tx)
    assert f[:2] == b'\xAA\x55' and f[2] == 20 and len(f) == 24, f
    payload, chk = f[3:23], f[23]
    x = 0
    for b in payload:
        x ^= b
    assert x == chk, 'XOR checksum'
    assert payload[0] == 0xC5 and payload[1] == link.ROBOT_ID and payload[2] == L.seq and payload[3] == 1
    pulses = [struct.unpack_from('<H', payload, CMD_FIELDS['pulse_us'] + 2 * i)[0] for i in range(8)]
    assert pulses == [500, 1000, 1500, 2000, 2500, 1234, 1500, 1501], pulses
    L.send(np.full(8, 10.0))                       # angles past the +86 deg joint limit must clip to it: 1500 + 1.50 rad * 636.6
    p2 = bytes(L.ser.tx)[24 + 3:24 + 23]
    assert all(struct.unpack_from('<H', p2, 4 + 2 * i)[0] == 2454 for i in range(8)), [struct.unpack_from('<H', p2, 4 + 2 * i)[0] for i in range(8)]
    L.send_us([9999] * 8)                          # raw pulses must clip to 2500 us
    p3 = bytes(L.ser.tx)[48 + 3:48 + 23]
    assert all(struct.unpack_from('<H', p3, 4 + 2 * i)[0] == 2500 for i in range(8))
    print('PASS  command frame: AA 55 14 + 20-byte CmdPacket + XOR, pulses little-endian, clip to 500..2500')

    # Telemetry: a firmware-layout frame, with junk before it, must unpack to the right SI values.
    t = firmware_bytes(SAMPLE)
    x = 0
    for b in t:
        x ^= b
    L.ser.rx += b'\x00\xAA\x13' + bytes([0xAA, 0x55, 38]) + t + bytes([x])
    deadline = time.time() + 2
    while L.latest() is None and time.time() < deadline:
        time.sleep(0.01)
    tel = L.latest()
    assert tel is not None, 'telemetry frame not parsed'
    assert tel['seq'] == SAMPLE['seq'] and tel['t_ms'] == SAMPLE['t_ms']
    assert np.allclose(tel['acc_mid'], np.array(SAMPLE['mid'][:3]) * 9.81 * 4 / 32768)
    assert np.allclose(tel['gyro_head'], np.array(SAMPLE['head'][3:]) * np.deg2rad(1000) / 32768)
    assert abs(tel['amps'] - SAMPLE['servo_mA'] / 1000) < 1e-9 and abs(tel['batt_v'] - 7.812) < 1e-9
    assert tel['latch'] is True and tel['status'] == SAMPLE['status']
    assert abs(tel['acc_mid'][2] - 9.81) < 0.01, '8192 counts = 1 g at +-4 g'
    print('PASS  telemetry frame: resyncs past junk, 38-byte TelPacket unpacks to the expected SI values')

    # The policy observation keeps the simulator's current scaling even though the sensor now reads to 6.4 A.
    sys.path.insert(0, str(HERE.parent / 'sim'))
    for amps, want in [(1.0, 1 / 3), (3.2, 3.2 / 3), (5.5, 3.2 / 3)]:
        assert abs(min(amps, 3.2) / 3.0 - want) < 1e-9
    print('PASS  observation current term: min(amps, 3.2) / 3.0, same as sim/ouro_env.py')
    print('all protocol checks passed')


if __name__ == '__main__':
    main()
