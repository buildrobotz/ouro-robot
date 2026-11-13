# Software

Three computers run three programs. Only one of them is clever.

| Where | Program | Language | Job | Rate |
|---|---|---|---|---|
| ESP32 on the robot | `firmware/ouro_robot` | Arduino C++ | Move 8 servos to the pulse widths it is told. Read 2 IMUs, current, latch, battery. Go limp when in doubt. | 200 Hz servo slew, 100 Hz telemetry |
| ESP32 on USB | `firmware/ouro_dongle` | Arduino C++ | Copy bytes between USB serial and ESP-NOW | as fast as they arrive |
| Your computer | `host/` and `sim/` | Python 3.10+ | Simulation, training, the policy, logging, calibration | 50 Hz control |

The robot never thinks. It follows pulse widths, reports what its sensors feel, and goes limp if the computer stops talking. All the learning and every decision happens in Python on your machine, so the robot stays cheap and the software stays easy to change.

## Install

You need Python 3.10 or newer. Training runs on a CPU.

```
python -m venv ouro-env && source ouro-env/bin/activate
pip install mujoco gymnasium "stable-baselines3>=2.3" tensorboard pyserial pillow manifold3d
```

The two sketches in `firmware/` are Arduino C++ for a classic ESP32. Flash `ouro_robot` to the board that rides on the robot and `ouro_dongle` to the one that stays plugged into your computer.

## Simulate and train first

Do this before you buy a single servo. It shows you what the robot will do and gives you a trained policy to bring to the hardware.

```
cd sim
python ouro_model.py                 # build ouro.xml
python test_gaits.py                 # hand-written gaits
python train.py roll                 # ~14 min on 10 cores
python train.py bite --steps 900000  # ~7 min
python evaluate.py roll 100
python stress.py
python limits.py
python demo.py
```

| Script | Use it to |
|---|---|
| `ouro_model.py` | Write `ouro.xml`, the MuJoCo model. |
| `test_gaits.py` | Run this first. It plays the hand-written motions and proves the model and the magnet latch work. |
| `train.py` | Train a behaviour with PPO. Policies land in `sim/policies/` as stable-baselines3 `.zip` files. |
| `evaluate.py` | Score a behaviour on N random builds, so you know it works on more than one robot. |
| `stress.py` | Break it on purpose: weak servos, heavy, slippery, late, shoved. Results are on the [Simulation and RL](learning.html) page. |
| `limits.py` | See how far each joint really swings. The electronics touch at +92°. |
| `demo.py` | Record the whole act, as shown on this website. |

`ouro_env.py` is the environment behind all of them: servo model, magnets, sensors, rewards and randomisation. `gaits.py` holds the hand-written motions, used by both the simulator and the real robot.

## Make the parts

```
cd ../cad && python make_parts.py && python check_fit.py
```

`make_parts.py` turns the parameters at the top of the file into an STL for every printed part. `check_fit.py` checks interference, sweeps the joints, and checks the latch, the wheel and printability. Run it after every edit. The other CAD scripts:

| Script | Does |
|---|---|
| `components.py` | MG90S, ESP32, IMU and the rest as dummy solids for fit checks |
| `assembly.py` | Places parts and components: flat strap, wheel, any pose |
| `export.py` | Writes STL, GLB and renders into `cad/out/` |

## Bring up the robot

Two small sketches help during assembly:

- `firmware/servo_center` holds one servo at 1500 µs, so you can fit each horn with the servo centred.
- `firmware/i2c_scan` lists the I2C addresses it finds, so you can check that both IMUs and the INA219 answer.

Then, from your computer, in this order:

```
cd ../host
python test_protocol.py              # no hardware needed
python calibrate.py PORT
python play_gait.py PORT wave 6
python sysid.py PORT
python run_policy.py PORT roll --gain 0.4 --log run1.npz
```

| Script | Use it to |
|---|---|
| `test_protocol.py` | Check `link.py` against the firmware's packet layout. No hardware needed. |
| `calibrate.py` | Find each servo's straight position and direction. Once per robot. Writes `host/calibration.json` (centre pulse and sign per servo). |
| `play_gait.py` | Run a `gaits.py` motion on the robot. |
| `sysid.py` | Measure loop delay and slew on your robot. Once per robot. It steps one outer joint 30° and times the gyro response. |
| `run_policy.py` | Run a trained policy on the robot and log the run. |

`--gain` scales the action range. Start at 0.4, watch, and raise it toward 1.0.

`PORT` is the dongle's serial port: `/dev/cu.usbserial-XXXX` or `/dev/cu.wchusbserial-XXXX` on macOS, `/dev/ttyUSB0` on Linux, `COM5` on Windows.

Once `sysid.py` has measured your robot, put its delay and slew, and your measured magnet pull-off force, into `OuroEnv._set_params()` as narrower ranges centred on your build. Then fine-tune in sim, which takes about 3 minutes:

```
python train.py roll --init policies/roll.zip --steps 500000
```

## What talks to what

```
                 sim/gaits.py ───────────────┐          (same functions, same numbers)
                                             ▼
 sim/ouro_env.py ──train.py──> policies/*.zip ──> host/run_policy.py ──> host/link.py
       ▲                                              ▲      │               │  USB serial 921600
       │ ranges from sysid                            │      │               ▼
 host/sysid.py <──────────────── telemetry ───────────┘      │        ouro_dongle (ESP32)
                                                             │               │  ESP-NOW
 host/calibrate.py ──> host/calibration.json ────────────────┘               ▼
                       (centre pulse and sign per servo)               ouro_robot (ESP32)
                                                                        │ PWM x8      ▲ I2C, GPIO, ADC
                                                                        ▼             │
                                                                     8 servos    2 IMU, INA219, latch, battery
```

Two rules keep sim and robot the same:

1. **One definition of a motion.** `gaits.py` returns eight joint angles in radians, positive = curl up. `test_gaits.py` feeds them to MuJoCo; `play_gait.py` feeds the same function to the robot.
2. **One definition of an observation.** `OuroEnv._frame()` in sim and `frame()` in `run_policy.py` build the same 22 numbers in the same order with the same scaling. If you change one, change the other.

`host/link.py` is the only file that knows about bytes.

## Safety behaviour

Every layer can stop the robot, and whenever something goes wrong the servos go limp.

| Condition | Who acts | What happens |
|---|---|---|
| No command for 300 ms | robot firmware | stops sending servo pulses; servos go limp |
| Battery under 6.6 V for 2 s | robot firmware | servos limp, LED blinks |
| Telemetry older than 0.2 s | `run_policy.py` | stops, sends "limp" |
| Current at the sensor limit for 2 s | `run_policy.py` | stops, sends "limp" |
| Any script exits or crashes | `link.limp()` in a `finally` block | servos limp |
| Command jumps | robot firmware | slew-limited to 10 rad/s |
| Angle past a joint limit | `link.py` | clipped to −69° … +86° before it is sent |
| Joint pushed past its command | the printed parts | end stop at +90°, before the electronics at +92° |

## Units and conventions

| Thing | Convention |
|---|---|
| Joint numbering | 1 = tail end, 8 = neck. Arrays are 0-indexed, so `q[0]` is joint 1. |
| Joint angle | radians, 0 = straight, positive = curl toward the back |
| Servo command on the wire | pulse width in microseconds, 500 to 2500 |
| Angle to pulse | `pulse = centre_us[j] + sign[j] × angle × 636.6` (2000 µs per 180°) |
| IMU axes | X toward the head, Y across, Z out of the back. Mount both boards that way. |
| IMU scaling | accel ±4 g, gyro ±1000 °/s, raw int16 on the wire, SI units after `link.py` |

## The two packets

Defined once in `ouro_common.h` (copied into both sketches) and mirrored in `host/link.py`. Little-endian, packed.

**Command**, computer → robot, 20 bytes, 50 Hz:

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | magic `0xC5` |
| 1 | u8 | robot ID |
| 2 | u8 | sequence number |
| 3 | u8 | flags: bit 0 = servos enabled |
| 4 | u16 × 8 | pulse width per servo, µs |

**Telemetry**, robot → computer, 38 bytes, 100 Hz:

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | magic `0x7E` |
| 1 | u8 | robot ID |
| 2 | u8 | sequence number |
| 3 | u8 | sequence number of the last command it received (for round-trip timing) |
| 4 | u32 | robot clock, ms |
| 8 | i16 × 6 | mid IMU: ax ay az gx gy gz |
| 20 | i16 × 6 | head IMU: ax ay az gx gy gz |
| 32 | i16 | servo rail current, mA |
| 34 | u16 | battery, mV |
| 36 | u8 | latch (1 = bitten) |
| 37 | u8 | status: bit 0 servos live, bit 1 low battery, bit 2 command timeout |

On USB serial each packet is wrapped as `0xAA 0x55, length, payload, XOR of payload`.

## Where to make changes

| To change | Edit |
|---|---|
| A hand-written motion | `sim/gaits.py` |
| What the policy sees | `OuroEnv._frame()` and `frame()` in `run_policy.py`, together |
| What the policy is rewarded for | the reward block in `OuroEnv.step()` |
| How bad the simulated servos are | `OuroEnv._set_params()` |
| A printed part | the parameters at the top of `cad/make_parts.py` |
| Pins, robot ID, Wi-Fi channel | `firmware/*/ouro_common.h` and the pin table at the top of `ouro_robot.ino` |

## Repository map

```
ouro-robot/
├── README.md               the whole project on one page
├── docs/                   detailed design: mechanical, electronics, software, parts
│   ├── bom.json            parts list with links and prices (the website reads this)
│   ├── electronics.json    netlist, pin map, wire list
│   └── tools/gen_electronics.py   draws schematic.svg, wiring.svg, perfboard.svg and checks them
├── cad/
│   ├── make_parts.py       every printed part, from parameters -> STL
│   ├── components.py       MG90S, ESP32, IMU ... as dummy solids for fit checks
│   ├── assembly.py         places parts and components: flat strap, wheel, any pose
│   ├── export.py           STL, GLB and renders into out/
│   ├── check_fit.py        interference, joint sweep, latch, wheel, printability
│   └── out/                STL, GLB and renders
├── sim/
│   ├── ouro_model.py       writes ouro.xml (the MuJoCo model)
│   ├── ouro_env.py         the environment: servo model, magnets, sensors, rewards, randomisation
│   ├── gaits.py            hand-written motions; used by BOTH sim and the real robot
│   ├── train.py            PPO training
│   ├── evaluate.py         score a behaviour on N random builds
│   ├── stress.py           break it on purpose: weak servos, heavy, slippery, late, shoved
│   ├── limits.py           how far each joint really swings; electronics touch at +92 deg
│   ├── demo.py             the whole act, recorded for the website
│   ├── test_gaits.py       first thing to run: proves the model and latch work
│   └── policies/           trained policies (.zip, stable-baselines3)
├── firmware/
│   ├── ouro_robot/         on-robot sketch + ouro_common.h (packet definitions)
│   ├── ouro_dongle/        USB dongle sketch + the same ouro_common.h
│   ├── servo_center/       assembly helper: hold one servo at 1500 us
│   └── i2c_scan/           bring-up helper: list I2C addresses
├── host/
│   ├── link.py             the only file that knows about bytes
│   ├── test_protocol.py    checks link.py against the firmware's packet layout, no hardware
│   ├── calibrate.py        find each servo's straight position and direction
│   ├── play_gait.py        run a gaits.py motion on the robot
│   ├── sysid.py            measure delay and slew on your robot
│   └── run_policy.py       run a trained policy on the robot
└── site/                   the project website (build_site.py -> dist/)
```
