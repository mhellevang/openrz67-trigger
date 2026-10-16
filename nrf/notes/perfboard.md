# Perfboard variant: sketch

Not built or tested. A do-it-yourself path for people who do not want to order assembled
boards from JLCPCB: off-the-shelf parts, through-hole soldering only, same Zephyr firmware and
GATT as the coin-cell board, so the apps do not change.

## Parts

| What | Note |
|---|---|
| Seeed XIAO nRF54L15 (not the Sense) | nRF54L15, antenna, LiPo charger, USB-C with a SAMD11 CMSIS-DAP on board: flashing needs no Pico probe |
| 2 × Panasonic AQY212EH | DIP-4 PhotoMOS, LED operate current 3.0 mA max, same threshold as the TLP172AM |
| 2 × 330 Ω | through-hole, 1/4 W |
| 1S LiPo, 100–300 mAh | on the XIAO's BAT+/BAT− pads underneath, charged over USB-C |
| Perfboard, about 30 × 40 mm | 2.54 mm pitch |
| Camera cable | as today: four wires, Dupont sleeves at the camera end |

Why a LiPo and not a CR2032: the XIAO runs the cell through its own charger and regulator,
built for a LiPo. A coin cell on 3V3 would bypass the regulator, and a USB cable plugged in at
the same time would push 3.3 V into the cell. The LiPo keeps it simple and USB-C charges it.

## Wiring

![Wiring diagram: XIAO D1 and D2 drive two AQY212EH relays through 330 Ω; the relays switch camera S1 and S2 to camera GND](perfboard-wiring.svg)

Every wire, as a checklist:

| From | To |
|---|---|
| XIAO D1 | 330 Ω, then AQY212EH #1 pin 1 |
| XIAO D2 | 330 Ω, then AQY212EH #2 pin 1 |
| XIAO GND | AQY212EH #1 pin 2 and #2 pin 2 |
| AQY212EH #1 pin 4 | camera pin 3 (S1) |
| AQY212EH #2 pin 4 | camera pin 4 (S2) |
| AQY212EH #1 pin 3 and #2 pin 3 | camera pin 2 (GND) |
| LiPo + / − | XIAO BAT+ / BAT− pads, underside |

Camera pin 1 (6 V) stays unconnected. Camera GND never touches the XIAO's GND, as on both PCBs.
The relay's switch side has no polarity, so pins 3 and 4 can swap.

Drive: the XIAO's GPIO sits at a regulated 3.3 V, so the LED current does not fall with the
battery. (3.3 − 0.4 − 1.5) / 330 ≈ 4.2 mA worst case, above the 3.0 mA threshold. Two channels
plus the on-board LED stay under Nordic's 15 mA GPIO total.

Wake button: the XIAO's own USR button (P0.00, SENSE-capable, `sw0` in the board's devicetree).
The case needs a hole for it. An external button can go from D0 (P1.04) to GND instead, the
same pin as the coin-cell board.

Status LED: the XIAO's LED is on P2.00, the same pin as the coin-cell board, but active low.
The board's `led0` alias already says so.

## Firmware

Same app as the coin-cell board. `firmware/boards/xiao_nrf54l15_nrf54l15_cpuapp.overlay`
adds the two outputs, points the battery reading at the XIAO's divider and turns on the
watchdog; the XIAO's own board files give `led0`, `sw0` and `watchdog0`. Builds with sysbuild
and MCUboot on NCS v3.4.1 (2026-10-15), not flashed:

```sh
west build -p -b xiao_nrf54l15/nrf54l15/cpuapp -s $FW -d $FW/build -- -DBOARD_ROOT=$FW -DSB_CONFIG_BOOT_SIGNATURE_KEY_FILE=\"$KEY\"
west flash -d $FW/build    # over the XIAO's USB-C
```

Battery: on the XIAO, VDD is the regulated 3.3 V, so the coin-cell board's VDD reading would
always say full. The LiPo sits behind a load switch (P1.15) and a 1:2 divider into AIN7. The
overlay gives the firmware the switch, the divider and a LiPo range (3.3 to 4.15 V) through
`zephyr,user`; the firmware turns the switch on for each reading only. The board's own
`vbat_pwr` regulator is disabled so it does not hold the switch on.

## Open

- **Battery reading.** The 1:2 divider comes from Seeed's sample (`× 2`), not the schematic.
  Check against a multimeter on the first build.
- **Idle current.** Seeed gives no board-level System OFF figure. The charger, regulator and
  whether the SAMD11 runs from the battery set it. Measure before promising battery life.
- **DC/DC.** The XIAO has the inductor and enables DC/DC in its devicetree. Check that it
  agrees with the firmware's `INDUCTORDET` check at boot.
- **Size.** The XIAO is 21 × 17.5 mm. With two DIP-4 parts and a LiPo, the box ends up about
  the size of the ESP32 case, not the coin-cell one.
