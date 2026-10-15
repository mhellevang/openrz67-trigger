# openrz67-trigger

Bluetooth LE remote release for the Mamiya RZ67. A small board plugs into the camera's
electrical remote port and is controlled from the companion apps,
[openrz67-android](https://github.com/openrz67/openrz67-android) and
[openrz67-ios](https://github.com/openrz67/openrz67-ios).

Two boards, same protocol, same apps:

| Directory | Board | Status |
|---|---|---|
| [`esp32/`](esp32/) | ESP32-C3, 250 mAh LiPo with USB-C charging, PlatformIO firmware | rev 2 in regular use |
| [`nrf/`](nrf/) | nRF54L15 module on a CR2032, no charger or regulator, Zephyr firmware | rev A ordered, not yet tested |

Both switch the camera through two isolated PhotoMOS relays. [`nrf/notes/perfboard.md`](nrf/notes/perfboard.md)
shows the same circuit on perfboard with off-the-shelf parts.

## Camera connection

Viewed from the front, the camera's four-pin remote-control port is:

1. 6 V, wired through, unused by the board
2. GND
3. S1 switch
4. S2 switch

To trigger the shutter, the PhotoMOS outputs short S1/S2 to camera GND. For bulb exposures,
set the shutter-speed dial to `B`. The camera closes the shutter by itself after about
60 seconds even if the switches stay closed.

## BLE protocol

The device advertises as `OpenRZ67` with service UUID `c9239c9e-6fc9-4168-b3aa-53105eb990b0`
and characteristic `458d4dc9-349f-401d-b092-a2b1c55f5319`. Send commands using Write Without
Response. The characteristic is open (no pairing): anyone in radio range can fire the camera,
which is accepted for a shutter release.

Also exposed: the standard Device Information Service (`0x180A`, firmware revision `0x2A26`),
and on boards that measure their battery (ESP32 rev 3, the nRF board):

* characteristic `cda71ce6-4af9-4aa2-8d34-329c2acdae09` (read + notify): battery voltage in
  millivolts, little-endian `uint16`.
* Battery Service (`0x180F`, Battery Level `0x2A19`): the same reading as a percentage, which
  is what phones show without any app. Each board's README gives its range.

**Single-byte commands** (value = button × 10 + state):

| Value | Action |
|-------|--------|
| 11 | Trigger shutter with a 100 ms pulse |
| 10 | Clear status LED; no shutter action |
| 21 | Start bulb mode: holds S1/S2 closed until 20 is sent. Camera dial must be on `B` |
| 20 | End bulb mode |
| 31 | Start countdown with the default duration of 10 s |
| 30 | Cancel countdown |

**Three-byte commands** `[command, duration, action]`:

| Bytes | Action |
|-------|--------|
| `[3, n, 1]` | Start countdown of *n* seconds (`1–255`) |
| `[3, _, 0]` | Cancel countdown |

Starting a trigger or bulb exposure cancels any pending countdown. Commands are queued in
the BLE callback and executed outside it, so the BLE stack never blocks on shutter timing.

## License

[MIT](LICENSE), covering the firmware, the KiCad design files and the enclosure sources for
both boards. MIT is written for software, but its grant is broad enough
to serve a hardware design; the closest hardware-specific equivalent is CERN-OHL-P.

Two things in this repository are **not** covered, because they are not mine to license:

* The footprints and symbols in `esp32/pcb/kicad/openrz67.pretty/`, `openrz67.kicad_sym`,
  `nrf/kicad/openrz67-nrf.pretty/` and `openrz67-nrf.kicad_sym` were imported from the
  EasyEDA/LCSC part libraries.
* The 3D models in `esp32/pcb/kicad/openrz67.3dshapes/` were fetched per LCSC part number with
  `easyeda2kicad`.

Both are redistributed here for convenience under their originators' terms.

## Credits

The camera connector pinout that made this project possible was described to me by
Julio Ryuuzaki, who built and sold the Bluetooth [RZ Blue](https://ryuuzaki.jp/) trigger
for years. He answered a cold email in December 2022 with the essentials — four pins, one
ground, two live, one unconnected — and the warning that the camera closes the shutter by
itself after about a minute in bulb mode. Thank you.
