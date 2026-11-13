// Shared by ouro_robot and ouro_dongle. Keep the two copies IDENTICAL (ouro_robot/ and ouro_dongle/).
// Every robot in a classroom needs its own ROBOT_ID; neighbours should use different channels (1, 6, 11).
// Byte layout is little-endian and packed; host/link.py mirrors it with struct formats
//   CmdPacket  '<BBBB8H'          (20 bytes)
//   TelPacket  '<BBBBI6h6hhHBB'   (38 bytes)
// host/test_protocol.py checks both sizes and offsets against this file.
#pragma once
#include <stdint.h>
#define ROBOT_ID      7
#define WIFI_CHANNEL  6
#define N_SERVO       8

#define CMD_MAGIC  0xC5
#define TEL_MAGIC  0x7E

// TelPacket.status bits
#define ST_LIVE       0x01   // servo pulses are being sent
#define ST_LOW_BATT   0x02   // pack under 6.6 V for 2 s (latched until reset; robot sleeps 30 s later)
#define ST_TIMEOUT    0x04   // no valid command for 300 ms
#define ST_RAIL_LOW   0x08   // servo rail under 4.5 V while live (buck in current limit / tripped, or pack sagging)
#define ST_MID_FAULT  0x10   // mid IMU (0x68) not answering: mid[] holds the last good sample
#define ST_HEAD_FAULT 0x20   // head IMU (0x69) not answering: head[] holds the last good sample
#define ST_INA_FAULT  0x40   // INA219 (0x40) not answering: servo_mA holds the last good value

struct __attribute__((packed)) CmdPacket {      // PC -> dongle -> robot, 50 Hz, 20 bytes
  uint8_t  magic;            // 0xC5
  uint8_t  robot_id;
  uint8_t  seq;
  uint8_t  flags;            // bit0 = servos enabled (0 = go limp)
  uint16_t pulse_us[N_SERVO];// 500..2500, already trimmed on the PC
};

struct __attribute__((packed)) TelPacket {      // robot -> dongle -> PC, 100 Hz, 38 bytes
  uint8_t  magic;            // 0x7E
  uint8_t  robot_id;
  uint8_t  seq;
  uint8_t  last_cmd_seq;     // lets the PC measure round-trip latency
  uint32_t t_ms;
  int16_t  mid[6];           // MPU-6050 #1 (0x68) raw: ax ay az gx gy gz  (+-4 g, +-1000 dps)
  int16_t  head[6];          // MPU-6050 #2 (0x69)
  int16_t  servo_mA;         // INA219, total servo rail, mA (range +-6.4 A with the added 0.1 ohm shunt)
  uint16_t batt_mV;          // 2S pack through the 100k/33k divider (filtered)
  uint8_t  latch;            // 1 = head pins bridged by the tail wire (debounced)
  uint8_t  status;           // ST_* bits above
};

static_assert(sizeof(CmdPacket) == 20, "CmdPacket must be 20 bytes");
static_assert(sizeof(TelPacket) == 38, "TelPacket must be 38 bytes");
