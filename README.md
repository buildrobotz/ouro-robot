# Ouro: the worm that bites its own tail and rolls away

Build a robot that crawls like a caterpillar, curls up, snaps its head onto its own tail with magnets and rolls away as a wheel. Eight $2.75 hobby servos, nine 3D-printed links, an ESP32 and a reinforcement learning policy you train on your own laptop in about 15 minutes.

Everything you need is in this repository: the MuJoCo simulation, the training code and trained policies, the Arduino firmware, the computer-side control code, the printable parts and a step-by-step build plan. The project website (built from this repo, live at ouro.buildrobotz.com) walks through it page by page.

> **AI-generated, not yet built.** This design, its code and its documentation were generated with AI, and nobody has built or tested a physical Ouro yet. The simulation, training code, trained policies, firmware, host code and print files are complete, the parts pass a full numerical fit check, and every performance number here comes from simulation across randomised builds. Treat it as a well-worked starting point: check each step, expect to fix things, and build at your own risk, especially around the LiPo battery. Section 9 lists what to measure and watch.

| Path | What it is |
|---|---|
| `sim/ouro_model.py` → `sim/ouro.xml` | The MuJoCo model (generator and generated MJCF) |
| `sim/ouro_env.py` | Gymnasium environment: servo model, magnet latch, sensors, three tasks, domain randomisation |
| `sim/gaits.py` | Hand-written motions (no RL): wave, cobra, bite, startle |
| `sim/train.py`, `sim/evaluate.py`, `sim/stress.py`, `sim/demo.py` | Train, score over randomised builds, stress-test, replay the whole act |
| `sim/policies/` | Trained `roll` and `bite` policies, ready to run |
| `firmware/ouro_robot/`, `firmware/ouro_dongle/` | Arduino sketches for the two ESP32s |
| `host/` | Computer side: serial link, calibration, gait player, policy runner, system identification |
| `cad/make_parts.py`, `cad/check_fit.py` → `cad/out/` | Printable parts generated from parameters, the fit check, STL and GLB files, renders |
| `docs/` | Detailed guides: mechanical, electronics, software, parts, stress tests |
| `site/` | The project website: `python site/build_site.py` writes `site/dist/` |
| `visual/ouro.html` | The motion replay on its own |

---

## 1. The robot

**Ouro** is a flat strap of nine printed links joined by eight MG90S servos, all bending in the same plane. Lying on the desk it is 432 mm long, 80 mm wide and 16 mm thick, about 350 g, and it looks like a watch strap or a piece of tank track with two googly eyes at one end. No chassis, no legs, no wheels.

Then it moves. It crawls forward like a caterpillar, stops, rolls itself up from both ends, snaps its head onto its tail with magnets and becomes a 140 mm wheel that rolls away by reshaping itself. Tap it and the wheel bursts open and flops flat. Nobody looking at the strap expects the wheel.

**Your first 30 seconds with a finished robot.** Power on and run `python play_gait.py PORT wave 6`: a hump travels down the strap and it creeps forward. Run `cobra` and the front three links rear up and nod. Run `bite` and it curls up and clicks shut. Then you find out the scripted bite only works on a little over half of all builds (36 of 60 randomised builds in sim). Getting from there to a bite and a roll that work every time is what the rest of the project teaches.

**What you will learn by building it.**

- Mechanical design with compliance: printed leaf springs, backlash and magnets as features rather than flaws.
- Electronics: a 2S LiPo power system, two buck converters, I2C sensors and a wireless ESP-NOW link.
- Simulation: a MuJoCo model of cheap servos that behaves as badly as the real ones.
- Reinforcement learning: writing observations and rewards, training PPO, and evaluating over many randomised robots.
- Sim to real: calibration, system identification and running a trained policy on hardware.

**Why this body works with cheap servos.** A closed loop of eight position servos is over-constrained: the eight commanded angles almost never agree with a closed shape, so stiff, accurate servos would fight each other to a stall. Ouro works because nothing in it is stiff or accurate. Every link has a 1 mm printed leaf spring in series with its servo, the gears have a few degrees of backlash, and the magnets capture from about 10 mm away. The slop is the tolerance that lets the loop close.

**What it cannot do:** steer. Every joint axis is parallel, so it lives on a line. The [Limits](risks.html) page covers this and the other trade-offs.

---

## 2. Bill of materials

The full list with product links, pack sizes and prices is on the [Parts](parts.html) page.

| Part | Per robot | About |
|---|---|---|
| MG90S metal-gear servo, 180° | 8 | $23 |
| ESP32 DevKit (ESP32-WROOM-32) | 2 | $14 |
| GY-521 MPU-6050 IMU | 2 | $5 |
| INA219 current sensor | 1 | $6.50 |
| Mini560 fixed 5 V 5 A buck | 2 | $5 |
| **2S 7.4 V 450 to 500 mAh LiPo, two-pack with charger (the one expensive component)** | 1 kit | $17 |
| XT30 connectors, 3 A polyfuse, shunt resistor, capacitors | | $6 |
| Magnets, O-rings, steel screws, zip ties, wire, three servo extensions, resistors, perfboard, header, ring lugs, PETG | | $10 |
| **Total per robot** | | **about $87** |

About $87 per robot assumes packs are split across a group, at typical Amazon pack prices. Ordering the servos and boards from AliExpress brings it to about $55 if you can wait 2 to 4 weeks for shipping. Buying only from Adafruit, Pololu and DigiKey costs about $170. One person buying every pack for a single robot spends about $235 and has spares left over. Prices move, so check before you order.

---

## 3. The design in MuJoCo

The model is generated by `sim/ouro_model.py` and written to `sim/ouro.xml`. Build it with `python ouro_model.py`. It compiles to 23 position coordinates, 22 degrees of freedom, 8 actuators and 349 g.

**Conventions.** Metres, kilograms, radians. x runs along the body from tail to head, y is across (all eight servo axes), z is up. Every segment's body frame sits on its **rear** joint axis. A positive joint angle curls the body upward, toward its back.

**Bodies.** Nine segments, `s0` (tail) to `s8` (head), in one nested chain. `s0` carries a free joint, so the robot can fall over sideways in sim just as it can on a desk.

| Segment | Mass | What is in it |
|---|---|---|
| `s0` (tail) | 36 g | Printed tail 19 g with two pairs of treads and the magnet cradle, MG90S 13.4 g, hardware |
| `s1`, `s2`, `s6`, `s7` | 31 g each | Printed link 13.6 g, MG90S 13.4 g, O-rings, zip tie, screws, wire ≈ 4 g |
| `s3` | 33 g + 34 g | Battery link, with the 2S pack standing on its long edge on the back |
| `s4` | 32 g + 12 g | ESP32 link, with the DevKit and the mid IMU |
| `s5` | 32 g + 16 g | Power link, with the perfboard: two bucks, INA219, capacitor, servo header |
| `s8` (head) | 20 g + 4 g | No servo, two steel pins, head IMU |

The electronics are modelled as boxes at the positions and sizes the mechanical design gives them (see [Mechanical](mechanical.html), where each board mounts), so they collide in sim where they would on the robot.

**Geometry per segment.** One box 37 × 76 × 16 mm for the link, and one cylinder of radius 9 mm and length 80 mm along y at its front joint axis (the tread cheeks sit on the front of the link that holds the servo; the tail has a second set at its rear). The cylinder is the O-ring tread and is the only thing that touches the floor. Pitch between joint axes is 48 mm. Closed, the nine links form a 9-gon with 40° at every corner and a circumradius of 70 mm.

**Joints.** Each of `s1` to `s8` has two hinges on the same axis:

- `j1` to `j8`: the servo. Range −1.20 to +1.50 rad (−69° to +86°). With up to 4° of backlash a link can reach +90°, where a printed end stop on the real part takes over; the limits are stiff in sim to match. At +92° the electronics on the middle links would touch. Damping 0.012 N·m·s/rad, armature 1.5e-4 kg·m², dry friction 0.006 N·m.
- `slop1` to `slop8`: an unactuated hinge with a hard range of ±2° (randomised 0.5° to 4° per joint). This is the gear backlash: the link can rattle freely inside that band around wherever the servo is holding.

**Actuators.** Eight `position` actuators on `j1` to `j8`, kp = 1.0 N·m/rad, force limited to ±0.15 N·m (1.5 kg·cm, a derated MG90S at 5 V). The low kp is deliberate. A bare servo stalls about 2° off target; the servo plus the 1 mm printed leaf spring in each link takes about 8°. That number comes from the leaf-spring design: `k = E·b·t³ / (12·l)` with E ≈ 2.5 GPa, b = 66 mm, t = 1.0 mm, l = 11 mm gives 1.25 N·m/rad, and in series with the servo's own stiffness about 1.0.

**The magnet latch.** Two `connect` equality constraints join the tail tip to the head tip at y = ±30 mm, which together make a free hinge across the width. They start inactive. The environment switches them on when the two latch points come within the capture radius (7 to 12 mm), and off again when the pull-out force along the tail's axis stays above the magnet strength (1.2 to 2.0 N) for a few tens of milliseconds. The pull is low-passed over 30 ms because magnets are a short-range well and a 2 ms knock does not separate them. Only pull-out releases it, because on the real part the pin sits in a notch that carries push and side loads in plastic.

**Sensors.** Accelerometer and gyro at `imu_head` (on `s8`) and `imu_mid` (on `s4`), in the same axes as the real boards when mounted with X toward the head and Z out of the back.

**Contacts.** Neighbouring segments do not collide with each other, and neither do head and tail (they are neighbours once latched). Everything else collides. Floor friction 0.9 nominal.

**Physics.** 2 ms timestep, `implicitfast` integrator, elliptic friction cones. The policy runs every 10 steps (50 Hz).

### Making the simulated servos as bad as real ones

A policy trained on perfect servos fails on cheap ones. So in `OuroEnv._apply_servo` and `OuroEnv._set_params`, every commanded angle goes through, in order:

1. **Latency**: a queue of 1 to 3 control steps (20 to 60 ms) for radio, USB and the servo's 20 ms PWM frame.
2. **Slew limit**: the internal target moves at 5 to 10 rad/s (0.10 to 0.21 s per 60°).
3. **Offset**: a fixed error of up to ±4° per joint, for horn-spline indexing and trim that calibration did not remove.
4. **Deadband**: inside 0.3° to 1.5° of target the servo stops correcting.
5. **Soft, weak actuator**: kp scaled 0.6 to 1.6, stall torque scaled 0.7 to 1.25.
6. **Backlash**: the free `slop` hinge, 0.5° to 4° per joint.

The simulated agent, like the real one, commands a position and never knows where the joint actually went.

---

## 4. The RL program

### Tasks

| Task | Start | Goal | Episode |
|---|---|---|---|
| `roll` | Closed wheel, random flat on the floor | Roll forward without bursting the latch | 8 s |
| `bite` | Flat on the belly | Bring head to tail and snap the magnets, stay a wheel | 6 s |
| `crawl` | Flat on the belly | Move forward while staying open | 8 s |

The included policies cover `roll` and `bite`. `crawl` is set up and left for you: the hand-written wave already moves 19 cm in 6 s, and that is the baseline to beat.

### Observation: only what the real robot can send

One frame is 22 numbers, and three frames are stacked (66 numbers):

| Signal | Size | What it tells the agent about what the body is actually doing |
|---|---|---|
| Head IMU accelerometer ÷ 9.81 | 3 | Which way is down for the head: how far round the wheel the head is, or how high it has reared |
| Head IMU gyro ÷ 5 | 3 | How fast the head is pitching. Roll rate of the wheel. |
| Mid IMU accelerometer ÷ 9.81 | 3 | Which way is down for the middle. Head minus mid is the overall curl. |
| Mid IMU gyro ÷ 5 | 3 | Pitch rate of the middle |
| Servo rail current ÷ 3 (clipped at 3.2 A) | 1 | Total effort. High and steady means servos are fighting the loop or stalled against the floor. |
| Latch bit | 1 | Whether it is a strap or a wheel |
| Its own previous action | 8 | What it asked for. With the stacked frames this lets it infer lag. |

No joint angles, no positions, no velocities from the simulator. Noise and per-episode bias are added to both IMUs and the current.

### Action

Eight numbers in [−1, 1], mapped to servo targets as `centre + scale × a`:

| Task | Centre | Scale | Reachable range |
|---|---|---|---|
| `roll` | 0.698 rad (40°, the 9-gon) | 0.60 | 5.6° to 74° |
| `bite` | 0.45 rad | 1.10 | −37° to 86° |
| `crawl` | 0 | 0.80 | ±46° |

### Reward

`roll`, per 20 ms step:

```
4.0 × clip(forward speed, −0.4, 0.25 m/s)      # go forward, but there is no prize above 25 cm/s
− 0.30 × current in amps                       # stop the servos fighting
− 0.02 × Σ(a − a_prev)²                        # smoothness, saves gears
− 1.0 × (latch pull ÷ magnet strength)²        # sim-only signal: do not strain the bite
− 40 and episode ends if the wheel bursts open
− 3 and episode ends if it falls on its side
```

`bite`, per step:

```
20 × (decrease in head-to-tail gap, metres)
+ 10 once, when the magnets snap
+ 1 for every step it remains a wheel
− 0.15 × current − 0.01 × Σ(a − a_prev)²
```

`crawl`: `6 × clip(speed, ±0.3) − 0.03 × current − 0.02 × smoothness`, −2 and end if it latches.

The latch-strain term uses a quantity the real robot cannot measure. That is allowed: rewards are computed in the simulator, and only observations have to exist on the robot.

### Reset

Every reset draws a new robot: masses ±15 %, servo stiffness, stall torque, backlash, offsets, deadband, slew rate, latency, floor friction 0.5 to 1.1, floor tilt ±2°, magnet strength and capture radius, IMU bias. `roll` starts the wheel at a random rotation; `bite` and `crawl` start flat with ±3° of pose noise. The model then settles for 0.2 s.

### Train it yourself

Libraries: `mujoco`, `gymnasium`, `stable-baselines3` (PPO, PyTorch on CPU), `numpy`, `tensorboard`, `pyserial`, `pillow`.

```
python -m venv ouro-env && source ouro-env/bin/activate
pip install mujoco gymnasium "stable-baselines3>=2.3" tensorboard pyserial pillow
cd sim
python ouro_model.py            # 1. build the model
python test_gaits.py            # 2. hand-written gaits: proves the model and the latch work
python train.py roll            # 3. PPO, all cores but two (about 12 min on 10 cores)
python train.py bite --steps 900000
tensorboard --logdir runs       # 4. watch it
python evaluate.py roll 100     # 5. score over 100 new random builds
python demo.py                  # 6. the whole act, saved to ../visual/
```

PPO settings (in `train.py`): 9 parallel environments, 8190 steps per update, batch 512, 6 epochs, γ = 0.98, λ = 0.95, learning rate 3e-4, entropy 0.002, two hidden layers of 128, initial log-std −0.7. Checkpoints every 500k steps in `runs/ckpt`.

If you are learning RL with this project, the two pieces to write yourself are the reward function and the observation frame: empty `_frame()` and the reward block of `step()` in `ouro_env.py`, keep `test_gaits.py` passing, and train.

### How to tell training is working

`train.py` prints one line per update:

```
  steps      time   dist     amps   latched   len
```

- `roll`: `dist` climbs from 0 toward about 2.5 m per 8 s episode, `latched` (fraction of episodes that end still a wheel) climbs toward 1, and `amps` falls toward the standing-wheel baseline. If `dist` rises but `latched` stays low, the policy has learned to throw itself forward and burst (see the lessons below).
- `bite`: `latched` climbs from about 0.1 past 0.9.
- A flat `dist` near zero after 500k steps means the reward or the observation is broken, not that it needs longer.

### What to expect

Trained on an Apple M1 Max laptop, 10 cores, CPU only:

| | `roll` | `bite` |
|---|---|---|
| Training | 2.0 M steps, 11.4 min | 0.9 M steps, 4.3 min |
| Scored on 60 new random builds | Still a wheel at 8 s: **60 of 60**. Distance 3.04 m mean, 2.56 m worst. Speed 38 cm/s. Mean current 2.7 A. | Latched at 6 s: **60 of 60**. Hand-written bite on the same builds: 36 of 60. |

The full act (`demo.py`: crawl, bite, 6 s of rolling, startle) on 20 random builds: bitten 20, still a wheel after rolling 20, mean roll 2.22 m, opened by the startle 17. **17 of 20 complete all of it.** The three misses are the hand-written startle not opening the latch from the pose the roll left it in; started from a resting wheel it opens 30 of 30. Small changes to the model (collision shapes, end stops, electronics boxes) leave these scores unchanged, so the behaviours do not hang on fine details. The stress tests below push them much further.

### Four lessons built into the design

Each of these showed up while developing the robot, and each one changed the design. They are worth knowing before you change anything.

1. **Stiff joints fight the loop.** With stiff joints (kp = 4), random actions drew 5 A because the closed loop fought itself. The leaf spring in every link exists to fix this.
2. **A policy will cheat if the reward lets it.** One policy learned to throw itself forward and burst the latch after about 4 s, and a larger burst penalty helped only slowly.
3. **The model was the problem, not the reward.** The latch released on any 2 ms force spike when a knuckle landed. Real magnets are a short-range well, so the pull is now low-passed over 30 ms. Rolling then loads the latch with about 1 N at most.
4. **That sets the magnet strength.** Stronger than 1 N so rolling holds, weaker than the 2.1 N the servos can reliably pull so the startle opens it. Hence 1.2 to 2.0 N, and the pull-off test in the build.

### Running the trained policy on the real robot

The policy never leaves the computer. `host/run_policy.py` loads `sim/policies/roll.zip`, and 50 times a second it reads the newest telemetry packet, builds the same 22-number frame as `OuroEnv._frame`, stacks three, calls `policy.predict`, converts the action to eight angles and then to eight pulse widths using `calibration.json`, and sends them through the dongle. It stops and lets the servos go limp if telemetry is older than 0.2 s or the current sits at the sensor limit for 2 s.

```
cd host
python calibrate.py PORT              # once per robot
python sysid.py PORT                  # once per robot
python run_policy.py PORT roll --gain 0.4 --log run1.npz
```

`--gain` scales the action range. Start at 0.4, watch, raise toward 1.0.

### Why RL works with servos that do not go where they are told

The policy does not learn "set joint 3 to 40°". It learns "when the head's gravity vector looks like this, the gyro reads that and the current is this, push these servos this way". The IMUs measure what the body did, whatever the gears did. Three signals carry it:

1. **Gravity direction at two points on the body.** A direct measurement of shape and rotation that does not pass through any servo.
2. **Gyro rate.** It shows within 20 ms whether the last command produced motion, so lag and stall are visible.
3. **Rail current.** Position error turns into torque and torque into current, so current is a cheap reading of "the body is not where the servos want it".

Because every training episode has different slop, offsets and lag, the policy cannot rely on any one servo behaving. It ends up closing the loop through the IMUs.

### Sim to real

| | |
|---|---|
| **Transfers from MuJoCo as is** | The overall strategy of each behaviour (roll up tips first, shift the shape to roll), the observation pipeline, the action mapping, the safety limits. |
| **Measured on your robot** | Servo centres and directions (`calibrate.py`). Loop delay and slew rate (`sysid.py`: steps one outer joint 30° and times the gyro response). Magnet pull-off force (kitchen scale, target 130 to 200 g, set by shimming the magnet deeper). Link mass (weigh one). |
| **Fine-tuned** | In sim, not on the robot. Put your measured delay, slew and magnet numbers into `_set_params()` as narrower ranges centred on your robot, then `python train.py roll --init policies/roll.zip --steps 500000`. About 3 minutes. |
| **Covered by domain randomisation** | Backlash 0.5° to 4°, offset ±4°, deadband, stiffness ×0.6 to ×1.6, stall torque ×0.7 to ×1.25, friction 0.5 to 1.1, latency 20 to 60 ms, mass ±15 %, tilt ±2°. The ranges are wide on purpose, so one policy file works across many slightly different builds. |
| **Not modelled, expect a gap** | Servo heating (torque fades over minutes), battery sag, the leaf springs' real stiffness and creep, O-ring grip on a dusty desk, cable drag across joints, the latch contact sensor bouncing. |

There is no reinforcement learning on the physical robot in the base project. A wheel that bursts open 2,000 times while exploring would wear out $22 of servos.

---

## 5. Electronics and ESP-NOW architecture

The full circuit is on the [Electronics](electronics.html) page: schematic, wiring map, power board layout, netlist, power budget, bring-up with a meter reading at each step, and troubleshooting. All four sketches compile without warnings on Arduino-ESP32 core 3.3.12 with ESP32Servo 3.0.9, board "ESP32 Dev Module". This is the summary.

```
 sensors ──I2C──> ESP32 on robot ──ESP-NOW──> ESP32 dongle ──USB serial──> computer
 (2 IMU, current,   (no processing)             (no processing)             Python: policy,
  latch, battery)                                                           logging, training
 servos <──8 PWM── ESP32 on robot <──ESP-NOW── ESP32 dongle <──USB serial── 8 pulse widths
```

**On the robot** (ESP32 DevKit on link `s4`, `firmware/ouro_robot`). Every 5 ms it moves each servo's pulse toward the last commanded value at no more than 10 rad/s. Every 10 ms it reads both IMUs, the INA219 and the latch, and broadcasts a 38-byte packet. If no command arrives for 300 ms it holds the servo signal lines low and the servos go limp. If the battery stays under 6.6 V it latches off and deep-sleeps 30 s later.

**Off the robot** (second ESP32 DevKit on USB, `firmware/ouro_dongle`). It copies ESP-NOW packets to serial and serial frames to ESP-NOW at 921600 baud.

**The link.** ESP-NOW broadcast on a fixed Wi-Fi channel with a robot ID byte in every packet. With several robots in one room, give each pair its own `ROBOT_ID` and spread neighbours across channels 1, 6 and 11 (`ouro_common.h`).

| Flow | Rate | Size |
|---|---|---|
| Telemetry, robot → computer | 100 Hz | 38 bytes |
| Commands, computer → robot | 50 Hz | 20 bytes |

**Latency budget**, command to first motion: serial under 1 ms, ESP-NOW 2 to 5 ms, wait for the servo's next 20 ms PWM frame 0 to 20 ms, servo dead time about 10 ms. Expect 25 to 50 ms; the sim randomises 20 to 60.

**Sensors.**

| Sensor | Where | Bus | Tells the agent |
|---|---|---|---|
| MPU-6050 #1 | `s4`, next to the ESP32 | I2C 0x68 | Orientation and pitch rate of the middle |
| MPU-6050 #2 | `s8`, head | I2C 0x69 (AD0 tied to 3V3) | Orientation and pitch rate of the head |
| INA219 with a second 0.1 Ω shunt in parallel | `s5`, in the servo 5 V line | I2C 0x40 | Total servo current. Reads to 6.4 A; the policy is given it clipped at 3.2 A, as in sim. |
| Latch contact | Head pins and tail cradle | GPIO 27 | Wheel or strap |
| Battery divider | 100 kΩ over 33 kΩ, 100 nF | GPIO 34 (ADC) | Pack voltage, for the cut-off only |

**Power.**

```
2S LiPo 7.4 V ── XT30 ── 3 A polyfuse ──┬── Mini560 A (5 V) ── INA219 ── servo rail (+ 220 µF low-ESR) ── 8 servos
                                        ├── Mini560 B (5 V) ── ESP32 5V pin
                                        └── 100k ─┬─ 33k ── GND
                                                  └── GPIO 34 (+ 100 nF)
```

**Pin map.**

| ESP32 pin | Goes to |
|---|---|
| 13, 26, 16, 17, 18, 19, 23, 25 | Servo signal, joint 1 (tail end) to joint 8 (neck) |
| 21, 22 | SDA, SCL (400 kHz) |
| 27 | Left head pin (right head pin to GND) |
| 34 | Battery divider midpoint |
| 2 | On-board LED |

**Design choices worth knowing:**

- The battery connector is XT30, not PH2.0. PH2.0 is rated about 2 A; rolling draws about 2.2 A from the battery continuously and a full stall about 5 A.
- A 3 A resettable fuse sits in the battery lead.
- The servo-rail capacitor is 220 µF low-ESR, not 1000 µF: the Mini560 can latch off at start-up into a large capacitor.
- A second 0.1 Ω resistor across the INA219 shunt doubles its range to 6.4 A.
- Servo 2 is on GPIO 26, not GPIO 14, because GPIO 14 pulses at boot.

**Power budget.** Rolling: about 2.2 A from the battery (2.7 A on the 5 V servo rail), about 12 minutes per pack. Standing as a wheel: 0.86 A, about 30 minutes. All servos stalled: about 5 A. After the low-battery cut-off the board still draws about 55 mA, so unplug the battery after every run.

---

## 6. Build plan

Plan on about 15 hours of hands-on work plus 9 to 10 hours of unattended printing. At an hour a day that is two to three weeks from ordering parts to a trained robot rolling on your desk: days 1 to 4 waiting for parts while you train in sim, days 5 to 12 building, the rest on the real robot. With parts and prints already on the desk, a long weekend is enough.

The detailed procedures live on the [Mechanical](mechanical.html) page (printing, one joint step by step, O-rings, magnets, where each board mounts) and the [Electronics](electronics.html) page (soldering the power board, bring-up, troubleshooting). This table is the order of work.

| # | Step | Hands-on | Waiting | Watch out for |
|---|---|---|---|---|
| 0 | Order parts (see [Parts](parts.html)) | 30 min | 2 to 5 days | Battery: 2S, 450 to 500 mAh, no thicker than 12.5 mm, with a charger. Mini560: the fixed 5 V version. ESP32: classic WROOM-32. Servos: 180°, not 360°. |
| 1 | While you wait: install Python and MuJoCo, run `ouro_model.py`, `test_gaits.py`, train `roll` and `bite` | 1 h | 12 min + 5 min of training on 10 cores; 1 to 2 h on a 4-core laptop | macOS: run the MuJoCo viewer with `mjpython`. |
| 2 | Measure one MG90S with calipers, edit the numbers at the top of `cad/components.py` and `cad/make_parts.py`, run `make_parts.py` and `check_fit.py`, print `coupon.stl`, test the fit | 1 h | 25 min print | **Expect one reprint.** Clone servos vary by ±0.5 mm in spline height and horn shape. The coupon checks the pocket, the horn recess and the pivot screw before you spend 9 hours printing. |
| 3 | Print 4 × `link`, `link_s3_battery`, `link_s4_esp32`, `link_s5_power`, `tail`, `head` in PETG, belly down, no supports | 30 min | 8 to 10 h | First-layer squish stiffens the 1 mm leaf spring (stiffness goes with thickness cubed). Check each spring flexes by hand and looks 1 mm thick. |
| 4 | Flash `firmware/servo_center` to one ESP32. Plug each servo into it in turn, let it centre at 1500 µs, fit the horn pointing along the link, number the servo | 40 min | | Arduino-ESP32 core 3.x is required. CH340 boards may need upload speed 460800. |
| 5 | Build one joint completely (Mechanical page, one joint step by step) and fold it by hand: it must swing to −69° and stop on plastic at +90° with the electronics untouched | 30 min | | Horn fitted one spline tooth off (about 15°). Refit so the link is straight at 1500 µs. |
| 6 | Build the other seven joints, fit the O-rings | 2 h | | Zip-tie tail left proud of the belly. Cut flush. |
| 7 | Latch: superglue two magnets into the tail (same pole out), drive the two M3 steel screws into the head with a ring lug under each, lay the bare sense wire in its groove across the tail notches | 40 min | 10 min glue | Stainless screws: a magnet will not hold them. |
| 8 | Measure latch pull-off with a kitchen scale: hook the head pins, pull the tail, read the peak. Target 130 to 200 g | 20 min | | Too strong and the startle cannot open it; too weak and the wheel bursts. Shim the magnets deeper with paper to weaken. |
| 9 | Flash `ouro_robot` to the robot's ESP32 and `ouro_dongle` to the other, **before cutting any pins**. Confirm telemetry arrives with nothing connected (`play_gait.py PORT wave 2`) | 40 min | | No telemetry: `ROBOT_ID` or channel differs between the two sketches. |
| 10 | Solder the power board (66 × 20.5 mm layout on the Electronics page). Fit XT30 and the 3 A polyfuse to the battery lead. Follow the bring-up steps: **both rails must read 5.0 V on a meter before anything is connected** | 2.5 h | | A "5 V" Mini560 that is really the adjustable version. A pack with a PH2.0 plug: it must be changed to XT30, one lead at a time. |
| 11 | Cut the ESP32's header pins flush, solder the signal and sensor wires to the stubs, mount battery, ESP32, IMUs and power board (Mechanical page, where each board mounts), route servo leads through the wire channels with a 15 mm slack loop at every joint, fit the three extensions | 2.5 h | | Wires taped across a leaf spring stop it flexing. Fix them to the solid blocks only. Run `firmware/i2c_scan`: it must list 0x40, 0x68, 0x69. |
| 12 | `calibrate.py`, then `wave`, `cobra`, `bite` on the desk | 1 h | | A joint that curls the wrong way: answer "n" in calibration, do not rewire. |
| 13 | `sysid.py`; put your delay and slew numbers into `_set_params()`; `python train.py roll --init policies/roll.zip --steps 500000` | 45 min | 3 min | Round trip over 30 ms: USB-serial driver buffering (FTDI latency timer). |
| 14 | `run_policy.py PORT roll --gain 0.4`, raise the gain | 1 to 2 h | | The wheel rocks but does not roll: friction. Clean the O-rings and the desk. |

**You are done when** the robot, starting flat, bites its tail, rolls at least 50 cm and opens on command, three times out of five, on video.

---

## 7. Learning path

If you want to learn robotics from this project rather than only build it, work through it in thirteen sessions of about an hour. Each session has one thing to do, one idea to learn and one check that tells you you are done for the day. It works for a self-learner, a club or a class.

| # | You do | You learn | Done for the day when | Most likely stuck on |
|---|---|---|---|---|
| 1 | Run `ouro_model.py`, open `ouro.xml` in the MuJoCo viewer, drag the links with the mouse | Bodies, joints, actuators; what an MJCF file is | You can point to joint 5 in the file and in the viewer | Viewer install |
| 2 | Edit `gaits.wave` (amplitude, frequency, wavelength), run `test_gaits.py` | A gait is a function of time. Open-loop control. | The wave moves more than 19 cm in 6 s | Sign of the travelling wave |
| 3 | Run the scripted `bite` on 6 random builds, watch it fail on some. Read `_set_params` | Why a script that works on the nominal robot fails on a real one. Backlash, offset, latency. | You can say which randomised parameter broke seed 1 | Believing the nominal run |
| 4 | Write `_frame()`: the observation, from sensor data only | Observability. What an IMU does and does not tell you. | The frame has 22 numbers and none comes from `qpos` | Wanting to peek at joint angles |
| 5 | Write the `roll` reward, train for 500k steps | Reward design. PPO in one page. | `dist` is rising | A policy that bursts the wheel for speed |
| 6 | Fix the reward, train to completion, run `evaluate.py roll 100` | Evaluating over a distribution, not one run | Latched at the end on over 90 % of builds | Reading one lucky episode as success |
| 7 | Print the coupon, measure a servo, fit it | Tolerances. Why you test the risky part first. | The servo drops in, the horn seats, the joint swings freely | Reprint |
| 8 | Assemble the eight joints and the latch, weigh the pull-off | Compliance as a design choice; magnets as a tolerance | Pull-off between 130 and 200 g | Horn one tooth off |
| 9 | Solder and wire power and sensors, flash both ESP32s | Power rails, brownout, I2C addresses, ESP-NOW | Telemetry prints on the computer, both rails at 5.0 V | Shared-rail resets |
| 10 | Calibrate, play `wave`, `cobra`, `bite` on the real robot | The same Python function drives sim and robot | The real wave moves; the real bite works sometimes | Joint direction |
| 11 | Run `sysid.py`, compare its numbers with the sim ranges | System identification. What "the gap" is, in numbers. | Your delay and slew sit inside the randomised ranges, or you widen them and retrain | USB latency timer |
| 12 | Run the trained `roll` at gain 0.4 and raise it; log a run and plot real IMU traces against sim | Sim to real. Reading a log. | The wheel completes one full turn | Friction, magnets too weak |
| 13 | Chain bite → roll → startle, film it | Switching policies on a sensor bit | A 20 s video of the full act | Latch bit bouncing |

**Done for the project:** the robot, starting flat, bites its tail, rolls at least 50 cm and opens on command, three times out of five, on video.

---

## 8. Three behaviours

**1. Beginner: the caterpillar and the cobra.** Two hand-written functions in `gaits.py`, no training. `wave` sends a hump from tail to head: 19 cm in 6 s in sim at about 1.8 A. `cobra` rears the front three links to put the eyes almost 10 cm off the desk and nods. You run them on screen in session 2 and on the desk in session 10, and they show that a gait is a function of time and that the same function runs in MuJoCo and on the robot.

**2. Real RL: the wheel.** Latched into a 9-gon, the robot has to roll by continuously changing its shape so that its centre of mass stays ahead of its contact point. It does this with two IMUs and a current reading, with joints it cannot see, without pulling its own magnets apart. There is no hand-written roll in the project: keeping a wheel rolling across slightly different builds is the job the policy learns. The trained policy rolls 3.04 m in 8 s on average across 60 randomised builds and never bursts.

**3. The demo video: the full ouroboros.** Crawl in from the left, stop, roll up tips first, bite, roll away as a wheel, then get tapped on the desk and burst open flat. The bite is a trained policy (60 of 60 randomised builds, against 36 of 60 for the script). The startle is a 0.6 s hand-written move: three joints at each end curl hard and the middle two bend backward, which pulls the pins straight out of the cradle. In sim it opens the latch on 30 of 30 resting wheels, with the end stops giving it something to pull against. The tap is detected on the computer as a spike in the mid IMU's accelerometer. `demo.py` plays the whole chain.

---

## 9. Limits and gotchas

Read this before you order parts. None of these stop the build, and each one is easier to handle when you know it is coming.

**It cannot steer.** All axes are parallel, so it goes forward and back along a line. A yaw joint in the middle would let the open worm turn but would be dead weight inside the closed wheel, so the design leaves it out. Adding one is a good extension project.

**It can fall on its side and cannot get up.** The wheel is 140 mm tall on an 80 mm base. On its side every joint axis is vertical and it is stuck; pick it up. The sim tilts the floor ±2° and ends the episode if it tips, but the model is left-right symmetric, so expect a few more sideways falls on a real desk than in sim.

**The servos fight each other and get hot.** This is the main wear mechanism. The standing wheel draws about 1 A doing nothing and the rolling wheel averages 2.7 A, which is every servo at roughly 45 % of stall. Keep wheel runs to a minute or two, let it cool, and treat servos as consumables. A 10-pack leaves two spares; for a class, stock four per robot.

**MG90S clones may be weaker than 1.5 kg·cm.** The sim randomises stall torque down to 1.05 kg·cm. The move that asks most of them is the bite, which lifts both ends off the desk. If your servos are weak, the bite stalls half way. Test before you build: one servo, one link, a 100 g weight at 10 cm.

**Print the coupon first.** The parts pass a full numerical fit check (servo fit, joint sweep, latch, wheel, electronics clearance), and a first print is where the remaining surprises show up. The known ones: servo and horn fit on clone servos, whose spline height and horn shape vary by ±0.5 mm (that is what `coupon.stl` is for); the 2 mm self-tapping pivot screw wearing its hole oval; and whether the head pins find the tail notch as the head swings in, which is checked seated but not on approach.

**The middle of the strap is full.** The battery, ESP32 and power board each fill the 32 mm of free back between their neighbours at full curl. Joints 4 to 6 touch electronics at +92°, so every joint has a printed end stop at +90°, and commands are capped at +86° in sim and in `host/link.py`. A battery thicker than 12.5 mm or a taller board will not fit without lowering that cap. The power board only fits with a Mini560 of 29 × 18 mm or smaller and an INA219 board no wider than 23 mm.

**Leaf springs fatigue.** Peak bending stress at stall is about 12 MPa. PETG should last; PLA will craze and snap at the root within hours of running. Print in PETG and print two spare links.

**Tune the latch on your robot.** The pull-off force between a 6 × 3 magnet behind 0.8 mm of plastic and an M3 screw shank is estimated at 2 to 4 N for the pair, and the design needs 1.2 to 2.0 N. Build step 8 measures it and you shim it down. The contact-based latch sensor may bounce while rolling; the firmware debounces 15 ms, and the Electronics page describes a hall-sensor fallback.

**The real wheel will be slower than the sim at first.** Friction, O-ring grip and servo heating are the hardest things to model. Expect the wheel to rock in place until you tune the gain and the magnets.

**The servo buck trips on over-temperature from about 6.5 A** and stays off until you replug the battery. The 3 A polyfuse protects against a short, not against hot servos.

**LiPo safety.** These packs have no protection circuit. The firmware cuts the servos at 6.6 V and deep-sleeps, but the board still draws about 55 mA, so unplug the battery after every run. If your pack comes with a PH2.0 or JST plug you have to fit an XT30, which means soldering on a live battery: one lead at a time, never both bare at once, or buy a pack that already has XT30. Charge in a LiPo bag, attended.

**Battery life is short:** about 12 minutes of rolling per pack, which is why the kit has two.

**Spares to keep per robot:** 2 servos (4 for a class kit), 2 printed links, 10 O-rings, 2 magnets, the second battery, zip ties.

**Why none of this is scary.** Every joint is the same part, so any failure is a five-minute swap of a $2.75 servo or a 15-gram print. The firmware goes limp on a lost link, low battery or timeout. The policy runner stops on stale telemetry or sustained over-current. Nothing on the robot is worth more than the battery.

---

## 10. Project website

`python site/build_site.py` writes `site/dist/`: nine static pages with relative links (overview with the motion replay, build plan, parts, mechanical with a 3D viewer, electronics, software, simulation and RL, learning path, limits). Copy the folder to any web host; it is meant for ouro.buildrobotz.com. Preview locally with `cd site/dist && python -m http.server`; the 3D viewer needs http, not `file://`. The motion replay on its own is `visual/ouro.html`, rebuilt by `python visual/build.py` from the trajectory `sim/demo.py` records.

---

## Licence

Code is MIT. Documentation, drawings, CAD and print files are CC BY-SA 4.0. Details and the no-warranty notice are in `LICENSE`.
