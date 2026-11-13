# Sourcing notes

The table above links one listing per part plus a cheaper or more trustworthy alternative. Listings come and go, so here is what to look for when the linked one is gone or looks wrong. Prices change; check before you order.

## Servos: MG90S, 180°

- The listing must say **180°**. A 360° or "continuous rotation" MG90S looks identical and will not hold a position. Skip listings that say "90-180°" without saying which, such as B0CH8BFXWW.
- Metal gears, about 2.0 kg·cm at 6 V. Buy 10 for one robot: two are spares, and servos wear out on this robot.
- The wiring plan assumes a 240 mm lead. Check the lead length in the listing. A shorter lead means longer extensions for the servos near the tail.
- Cheaper: Waveshare sells them at $2.59 each, $2.15 each at 10 to 49, shipped from China.
- More trustworthy: DFRobot's DMS-MG90 at $5.90 each ($5.20 at 10+), 180° ±10°, 12 g. Its lead is 180 mm, shorter than the 240 mm the wiring assumes.

## ESP32 boards

- **Classic ESP32-WROOM-32**, not C3, S2 or S3. The classic chip has the 16 PWM channels the firmware uses.
- The listed board is the 38-pin DevKit with USB-C and a CP2102 USB chip. Some similar listings are 30-pin or Micro-USB; check the photos.
- You need two per robot: one rides on the robot, one is the USB dongle. A 2-pack is about $14.
- Alternatives: DIYables' 38-pin USB-C board, about 55 × 28 mm, at $8.99 each. Adafruit's HUZZAH32 Feather ($19.95) is a genuine WROOM-32 but a different shape, so it will not drop into the same layout.

## IMUs: GY-521 MPU-6050

- Two per robot. A 5-pack is about $13, which covers two robots with a spare.
- Adafruit's MPU-6050 ($12.95) and RobotShop's ($13.64) are fine sensors but different board layouts from the GY-521.
- Mount both with X toward the head and Z out of the back.

## Current sensor: INA219

- Look for a 0.1 Ω shunt (marked "R100"). The common GY-219 board is rated 26 V, ±3.2 A; with a second 0.1 Ω resistor across the shunt it reads to 6.4 A.
- **Size matters.** The power board fits an INA219 up to 20.5 × 22.9 mm. Adafruit #904 ($9.95, 20.3 × 22.9 mm) fits beside full-size bucks. A 25.6 × 20.5 mm generic board fits only if your Mini560s are at most 26 mm long.
- Elecrow's INA219 module ($15) is a Raspberry Pi Pico-format board and costs more.

## 5 V regulators: Mini560

- **Fixed 5 V only.** Mini560 also comes in 3.3, 9 and 12 V versions, so check the "5V" marking. There is no fixed 6 V version, and an adjustable buck is the trap: a mis-set adjustable buck is how a 9 V rail kills eight servos.
- 7 to 20 V in, 5 A out, at most 29 × 18 mm. Expect it to need cooling above about 4 A; stalls must stay short.
- Check the output with a meter before you connect anything.
- A 5-pack is about $12 and covers two robots with one spare. Cheap Mini560s are easiest to find on eBay and AliExpress.
- The trustworthy fallback is Pololu's D24V50F5 at $32.95 each.

## Battery and charger

- **2S 7.4 V, 450 to 500 mAh.** Look for a kit with two packs and a USB balance charger.
- **No thicker than 12.5 mm.** The pack stands on its long edge on link s3, and anything thicker eats the clearance for joint 4. Packs sold as 2S 500 mAh range from 47 × 25 × 10 to 56 × 28 × 13 mm, so check. A slightly longer pack is fine.
- The listed URGENEX 450 mAh pack is about 46 × 25 × 9.7 mm and 23 to 25 g, and the kit comes with a 1-to-2 USB charger. The same brand also sells the packs without a charger (B0H7R5R7HS); that is not the one you want.
- **Fit XT30 connectors.** Small packs ship with PH2.0 or "JST" plugs. PH2.0 is rated about 2 A, and the robot draws 1.8 A continuously while rolling and 5 A in a stall. Put a female XT30 on each battery and a male on the robot. "JST" on a hobby pack usually means the larger RCY/SYP style, not PH2.0; with XT30 fitted it does not matter which you get.
- Alternatives from hobby shops: Tattu 2S 450 mAh 75C ($7.79, 44 × 25 × 14 mm, 29 g, which is thicker than the 12.5 mm limit) and ManiaX 2S 450 mAh 30C ($8.99, 62 × 30 × 11 mm, 31 g), plus an HPI 2S USB balance charger ($11.99). Stock comes and goes.

## Magnets

- **6 × 3 mm neodymium discs**, two per robot. Many packs sold as 6 × 3 are really 6 × 2.6 mm or thinner; check the listed thickness.
- 1/4 × 1/8 inch discs (6.35 × 3.18 mm) work too, after you change the pocket size in `cad/make_parts.py`. Apex Magnets sells an adhesive-backed 20-pack (10 N, 10 S) for $9.99; K&J Magnetics' D42 (N42) is about $0.41 each.
- Aim for a pull-off force of 130 to 200 g on a kitchen scale, and set it by shimming the magnet deeper.

## O-rings (the treads)

- **NBR, exactly 18 mm inside diameter and 2.5 mm cross-section.** 18 per robot. The nearest inch size, AS568-017, is too thin.
- They are hard to find on Amazon; some assortment kits include a few. The O-Ring Store sells N2.50X018 (70 durometer) at $0.08 each with an 85-piece minimum ($6.80), enough for four robots. Tameson sells a 100-pack.

## Screws

- **M3 × 25 mm socket head, carbon or alloy steel** (black oxide or zinc plated), because a magnet has to hold them. Not stainless.
- About $8 to $10 per 100. Pololu's M3 × 25 is a Phillips pan head, which is the wrong shape.

## Small electronics

- **Capacitor:** 220 µF 16 V low-ESR. Not 1000 µF: the Mini560 can latch off at start-up into more than about 440 µF.
- **Resistors:** you need 100 kΩ and 33 kΩ for the battery divider. Check the value table of any kit; the Aniann 1280-piece kit lists both, and smaller kits may not include 33 kΩ. DigiKey sells singles at about $0.10 each, plus shipping.
- **Perfboard:** double-sided 4 × 6 cm with plated-through holes. Adafruit's 3-pack ($2.50, 14 × 20 holes) does not say whether it is plated through.
- **Header pins:** 2.54 mm male, one 40-pin strip per robot. DFRobot sells 10 strips for $3.10; Pololu sells single strips at $0.95.
- **Servo extensions:** 3-pin JR/Futaba. DFRobot sells 150 mm ones at $0.60 each.
- **XT30 pairs, 3 A-hold polyfuse (such as RUEF300), 0.1 Ω 1 W resistor, 100 nF ceramic capacitor, M3 ring terminals:** any listing that matches the spec in the table.

## Wire, ties and filament

- **Wire:** silicone insulation in 22 AWG (red and black, for battery and servo power), 26 AWG and 28 AWG (signals). Adafruit's 25 ft spool of 26 AWG is $4.95, one colour per spool.
- **Zip ties:** 2.5 × 100 mm. ServoCity sells 100 for $4.99.
- **Filament:** PETG, about 150 g per robot. Elegoo's Rapid PETG is $13.99 per kg. Not PLA: the leaf springs fatigue.

## Hall sensor (only if you need it)

The A3144 hall switch is a fallback in case the contact latch sensor proves flaky, and is not in the per-robot price. The bare A3144 is obsolete, so listings vary. If you want the bare part, avoid listings for the "linear hall" version (such as B0FXX97T3F) and KY-003 modules. Ready-made hall modules from M5Stack and Adafruit also exist.
