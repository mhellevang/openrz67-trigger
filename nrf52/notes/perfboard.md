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

```
XIAO D1 (P1.05) ── 330 Ω ── AQY212EH #1 pin 1 (LED +)
XIAO GND ──────────────────  AQY212EH #1 pin 2 (LED −)
                             AQY212EH #1 pins 3/4 ── camera S1 (pin 3) and camera GND (pin 2)

XIAO D2 (P1.06) ── 330 Ω ── AQY212EH #2 pin 1 (LED +)
XIAO GND ──────────────────  AQY212EH #2 pin 2 (LED −)
                             AQY212EH #2 pins 3/4 ── camera S2 (pin 4) and camera GND (pin 2)

Camera pin 1 (6 V): not connected.
```

Camera GND stays separate from the XIAO's GND, as on both PCBs. The PhotoMOS output is
bidirectional, so pin 3/4 order does not matter.

Drive: the XIAO's GPIO sits at a regulated 3.3 V, so the LED current does not fall with the
battery. (3.3 − 0.4 − 1.5) / 330 ≈ 4.2 mA worst case, above the 3.0 mA threshold. Two channels
plus the on-board LED stay under Nordic's 15 mA GPIO total.

Wake button: the XIAO's own USR button (P0.00, SENSE-capable, `sw0` in the board's devicetree).
The case needs a hole for it. An external button can go from D0 (P1.04) to GND instead, the
same pin as the coin-cell board.

Status LED: the XIAO's LED is on P2.00, the same pin as the coin-cell board, but active low.
The board's `led0` alias already says so.

## Firmware

The app picks up `boards/xiao_nrf54l15_nrf54l15_cpuapp.overlay` on its own when built with
`west build -b xiao_nrf54l15/nrf54l15/cpuapp`. `src/main.c` needs `s1_drv`, `s2_drv`, the
`led0`, `sw0` and `watchdog0` aliases and one ADC channel in `zephyr,user`; the XIAO's board
files supply `led0` and `sw0`.

```dts
#include <zephyr/dt-bindings/adc/nrf-saadc.h>
#include <zephyr/dt-bindings/gpio/nordic-nrf-gpio.h>

/ {
	outputs {
		compatible = "gpio-leds";
		s1_drv: s1 {
			gpios = <&gpio1 5 (GPIO_ACTIVE_HIGH | NRF_GPIO_DRIVE_S0H1)>;	/* D1 */
		};
		s2_drv: s2 {
			gpios = <&gpio1 6 (GPIO_ACTIVE_HIGH | NRF_GPIO_DRIVE_S0H1)>;	/* D2 */
		};
	};

	aliases {
		watchdog0 = &wdt31;
	};

	zephyr,user {
		io-channels = <&adc 0>;
	};
};

&gpio0 {
	sense-edge-mask = <(1 << 0)>;	/* USR button via SENSE, as on the coin-cell board */
	latch-detect;
};

&wdt31 {
	status = "okay";
};
```

## Open

- **Battery level.** On the XIAO, VDD is the regulated 3.3 V, so the current VDD reading is
  always "full". The battery sits behind a load switch (`vbat_pwr`, P1.15) and a divider
  into AIN7 (P1.14); Seeed's sample multiplies by 2. Needs a small firmware change: switch on,
  sample AIN7, switch off, and a LiPo curve instead of the CR2032 one.
- **Idle current.** Seeed gives no board-level System OFF figure. The charger, regulator and
  whether the SAMD11 runs from the battery set it. Measure before promising battery life.
- **MCUboot.** The XIAO's board files include the standard nRF54L15 partitions, so sysbuild
  with MCUboot should build. Not tried.
- **DC/DC.** The XIAO has the inductor and enables DC/DC in its devicetree. Check that it
  agrees with the firmware's `INDUCTORDET` check at boot.
- **Size.** The XIAO is 21 × 17.5 mm. With two DIP-4 parts and a LiPo, the box ends up about
  the size of the ESP32 case, not the coin-cell one.
