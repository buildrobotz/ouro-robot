## How hard you can push it

A policy that only works on the exact robot it was trained on is no use to you, because your robot will not be that robot. `python sim/stress.py` makes one thing worse than anything in the training range and runs 30 new randomised builds under it. Each row below is one of those conditions. "Servo load" is the average torque as a share of stall torque while rolling.

| Condition | Wheel kept | Rolled in 8 s | Tipped over | Current | Servo load (mean / worst joint) | Bite closed |
|---|---|---|---|---|---|---|
| As trained | 30/30 | 3.08 m | 0 | 2.6 A | 45 % / 56 % | 30/30 |
| Servo stall torque 1.0 kg·cm (two thirds of nominal) | 30/30 | 2.93 m | 0 | 3.2 A | 58 % / 68 % | 30/30 |
| Servo stall torque 0.8 kg·cm (about half) | 30/30 | 2.58 m | 0 | 4.2 A | 68 % / 78 % | 19/30 |
| Mass +25 % | 30/30 | 3.78 m | 0 | 2.6 A | 47 % / 59 % | 30/30 |
| Mass +40 % | 29/30 | 4.16 m | 0 | 2.9 A | 49 % / 61 % | 30/30 |
| Friction 0.3 (dusty desk) | 30/30 | 3.08 m | 0 | 2.8 A | 45 % / 57 % | 30/30 |
| Loop latency 100 ms | 30/30 | 2.42 m | 0 | 2.7 A | 44 % / 55 % | 30/30 |
| Battery 10 mm further off centre | 30/30 | 3.13 m | 0 | 2.8 A | 45 % / 57 % | 30/30 |
| Desk tilted 5° sideways | 30/30 | 3.12 m | 0 | 2.7 A | 45 % / 57 % | 30/30 |
| Sideways shove, 0.5 N every second | 30/30 | 3.09 m | 0 | 2.6 A | 45 % / 57 % | 30/30 |
| Sideways shove, 1.5 N every second | 30/30 | 1.01 m | 8 | 2.6 A | 46 % / 58 % | 29/30 |
| Magnets 1.0 N (weak) | 30/30 | 3.08 m | 0 | 2.6 A | 45 % / 56 % | 30/30 |
| Magnets 3.0 N (strong) | 30/30 | 3.08 m | 0 | 2.6 A | 45 % / 56 % | 30/30 |

What this means for your build:

- **Cheap, sloppy parts are fine.** Two-thirds-strength servos, 40 % extra weight, a slippery desk and 100 ms of lag all still roll and bite. The first thing to fail is the bite with half-strength servos (19 of 30), because it lifts both ends off the desk and needs the torque. If your robot rolls but struggles to bite, suspect weak or worn servos first.
- **It is hard to knock over.** It takes a sideways shove of nearly half its own weight to tip it, and even then only 8 times in 30. A 5° sideways slope is fine. Magnet strength anywhere from 1 N to 3 N makes no difference to rolling.
- **The servos run warm whatever you do.** Rolling holds every servo at about 45 % of stall, and the worst joint above half. A policy rewarded for going slowly and drawing less current rolled at 14 cm/s instead of 38, and its servo load was no lower, so it is not in the repo. Letting two servos go limp while it is a wheel cut standing current only from 1.0 A to 0.8 A. The load comes from holding a nine-sided shape against gravity with soft joints, and no policy makes it go away. Plan for short runs, let the servos cool between them, and treat servos as consumables. That is why the parts list has two spares.

## Joint limits: does anything swing into the electronics?

`python sim/limits.py`. The electronics on the middle links touch at +92°. The printed end stop is at +90°. This is the largest true joint angle (servo plus backlash) seen on any of 30 random builds, per behaviour:

| Behaviour | j1 | j2 | j3 | j4 | j5 | j6 | j7 | j8 |
|---|---|---|---|---|---|---|---|---|
| Roll (policy) | 74 | 86 | 67 | 88 | 73 | 79 | 80 | 85 |
| Bite (policy) | 77 | 81 | 58 | 71 | 77 | 73 | 55 | 67 |
| Bite (script) | 52 | 53 | 52 | 45 | 43 | 52 | 53 | 54 |
| Wave | 34 | 33 | 32 | 46 | 35 | 32 | 34 | 36 |
| Cobra | 7 | 8 | 11 | 13 | 8 | 47 | 29 | 5 |
| Startle from a wheel | 89 | 90 | 90 | 89 | 90 | 89 | 90 | 88 |

Every normal behaviour stays clear of the stops. The startle lands on the end stops at every joint, which is what they are for. One thing to watch: the stop limits the servo's rotation, but the next link's leaf spring can still flex a few degrees beyond it under load. If you change the electronics layout or the link shape, keep that margin in mind and rerun `check_fit.py` and `limits.py`.
