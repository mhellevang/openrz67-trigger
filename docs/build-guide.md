# Build guide

Nothing is soldered by hand. The board comes assembled, everything else plugs in.

## Parts

| What | Where | Note |
|---|---|---|
| Assembled board | JLCPCB | see step 1 |
| CR2032 | anywhere | one per board |
| SH1.0 4P to Dupont female cable (Qwiic / STEMMA QT pigtail) | AliExpress | three: one camera cable (`J1`), one for flashing (`J2`), one spare |
| Raspberry Pi Pico 2 WH | any Pi reseller | the flashing probe. H = headers pre-soldered, W is irrelevant here. A Pico H works the same |
| ABS and a little clear filament | | for the case and the light pipe |

## 1. Order the board

These files are rev B. It fixes the two known faults in rev A, but is not tested yet.
Rev A and its faults are in [`kicad/archive/`](../kicad/archive/).

Upload these three files from [`kicad/out/`](../kicad/out/) to JLCPCB and pick assembly on
both sides:

- `openrz67-nrf-gerber.zip`
- `openrz67-nrf-bom.csv`
- `openrz67-nrf-pos.csv`

The schematic is [`openrz67-nrf-schematic.pdf`](../kicad/out/openrz67-nrf-schematic.pdf).

## 2. Print the case

```sh
cd case
./export.sh
```

This writes the STLs to `case/stl/` and a QIDI Studio project. It needs
[uv](https://docs.astral.sh/uv/). Print the shell and the floor in ABS, no supports. Print the
light pipe in clear filament and glue it into the shell. Details in
[`case/README.md`](../case/README.md).

## 3. Flash the firmware

Build it with the nRF Connect SDK, then flash it once over `J2` with the Pico as a probe.
The cell must be out while the probe is connected. After that, updates go over Bluetooth.
Step by step, with wiring: [`firmware/README.md`](../firmware/README.md).

## 4. Make the camera cable

`J1` has the camera port's pin order, so the cable is straight: pin 1 to pin 1. Push the
pigtail's Dupont ends onto the camera pins and tape them.

| `J1` / camera pin | Signal | Wire on a typical pigtail |
|---|---|---|
| 1 | 6 V, not used by the board | red |
| 2 | GND | black |
| 3 | S1 | blue |
| 4 | S2 | yellow |

Wire colours vary between cables. Go by position: pin 1 is next to the white dot on the
connector. Check the orientation before you plug it into the camera.

Rev A boards have a different `J1` order, see [`kicad/archive/`](../kicad/archive/).

## 5. Put it together

Slide the CR2032 into the clip, **+ side toward the floor** (there is a + on the floor).
Slide the floor in until it clicks. To change the cell later, see
[`case/README.md`](../case/README.md).

## 6. Install the app

Android: the APK from [releases](https://github.com/mhellevang/openrz67-android/releases/latest).
iPhone: build [openrz67-ios](https://github.com/mhellevang/openrz67-ios) in Xcode.

Press the button on the trigger. The app finds it and connects on its own.
If something does not work, see [support](support.md).
