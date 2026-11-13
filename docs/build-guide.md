# Build guide

Nothing is soldered by hand. The board comes assembled, everything else plugs in.

## Parts

| What | Where | Note |
|---|---|---|
| Assembled board | JLCPCB | see step 1 |
| CR2032 | anywhere | one per board |
| SH1.0 4P to Dupont female cable (Qwiic / STEMMA QT pigtail) | AliExpress | three: one camera cable (`J1`), one for flashing (`J2`), one spare |
| Raspberry Pi Pico 2 WH | any Pi reseller | the flashing probe. H = headers pre-soldered, W is irrelevant here. A Pico H works the same |
| Micro-USB cable | anywhere | from the Pico to the computer. A data cable, not a charge-only one |
| Filament, plus a little clear | | for the case and the light pipe |

## 1. Order the board

These files are rev B. It fixes the two known faults in rev A, but is not tested yet.
Rev A and its faults are in [`kicad/archive/`](../kicad/archive/).

Upload these three files from [`kicad/out/`](../kicad/out/) to JLCPCB and pick assembly on
both sides:

- `openrz67-nrf-gerber.zip`
- `openrz67-nrf-bom.csv`
- `openrz67-nrf-pos.csv`

The round pad in the middle of the back is the cell's minus contact. It has no part on it, but
it needs its solder paste. In the order remark, ask JLCPCB to print paste on that pad as the
paste layer shows.

The schematic is [`openrz67-nrf-schematic.pdf`](../kicad/out/openrz67-nrf-schematic.pdf).

## 2. Print the case

The files are in [`case/stl/`](../case/stl/):

| File | What | How it prints |
|---|---|---|
| `openrz67-nrf-lid.stl` | the shell | upside-down |
| `openrz67-nrf-lid-text.stl` | the text in the shell's top | a second colour, loaded as a part of the shell. Without it the text is a 0.8 mm recess |
| `openrz67-nrf-base.stl` | the floor | floor-down |
| `openrz67-nrf-lightpipe.stl` | the light pipe | head-down, clear filament |

No supports. The fits are tuned in ABS, but PLA or PETG should work too. Glue the light pipe
into the shell. Details, and how to change the case, in [`case/README.md`](../case/README.md).

## 3. Flash the firmware

Build it with the nRF Connect SDK, then flash it once over `J2` with the Pico as a probe.
The cell must be out while the probe is connected. After that, updates go over Bluetooth.
Step by step, with wiring: [`firmware/README.md`](../firmware/README.md).

## 4. Make the camera cable

`J1` has the camera port's pin order, so the cable is straight: pin 1 to pin 1. Tape the
pigtail's four Dupont ends together side by side, so they plug onto the camera as one.

| `J1` / camera pin | Signal | Wire on a typical pigtail |
|---|---|---|
| 1 | 6 V, not used by the board | red |
| 2 | GND | black |
| 3 | S1 | blue |
| 4 | S2 | yellow |

Wire colours vary between cables. Go by position: pin 1 is next to the white dot on the
connector. On the camera, seen from the front, pin 1 is the leftmost, under the small triangle.
Check the orientation before you plug it into the camera.

Rev A boards have a different `J1` order, see [`kicad/archive/`](../kicad/archive/).

## 5. Put it together

Before the first cell, wipe the round pad on the back of the board with isopropyl alcohol.
Flux left on it can keep the cell from making contact.

Slide the CR2032 into the clip, **+ side toward the floor** (there is a + on the floor).
Slide the floor in until it clicks. To change the cell later, see
[`case/README.md`](../case/README.md).

## 6. Install the app

Android: the APK from [releases](https://github.com/mhellevang/openrz67-android/releases/latest).
iPhone: build [openrz67-ios](https://github.com/mhellevang/openrz67-ios) in Xcode.

Press the button on the trigger. The app finds it and connects on its own.
If something does not work, see [support](support.md).
