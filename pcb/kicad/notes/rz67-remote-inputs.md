# RZ67: S1/S2 and contact resistance

Investigated on 2026-09-05. Documentation only, not production approval. No components changed.

## Conclusion

The manual describes the functions, contact labels, and camera battery current, but no specification was found for S1/S2 input current, threshold voltage, or maximum contact resistance. The online research likewise found no verified measurements of these values. This does not mean the information has never been published.

Low contact resistance is a reasonable way to reduce uncertainty. The TLP172AM is a better candidate than the TLP172GM in terms of voltage drop, but this alone does not establish camera compatibility. A final selection also requires checks of current, voltage and transients, temperature, leakage, timing, and LED drive.

## What the original manual actually says

Source: [Mamiya RZ67 repair manual](/Users/mathiashellevang/Koofr/Manualer/Mamiya-RZ67-repair.pdf). Page numbers below refer to the printed page numbers. The entire document was searched using OCR, and the relevant pages were also inspected visually. OCR may miss text.

| Page | Information | Relevance to the trigger |
|---|---|---|
| 95, fig. 114 | VS at viewfinder contact 2: 3.2 V ±2 mV. Appears when S1 is activated with an RZ lens; adjusted with VR1. | Reference output to the viewfinder, not the S1/S2 threshold or permitted voltage drop across the switch. |
| 102 | The winder's 9 V is converted to approximately 6 V, including for the RC connector. | Do not assume that BW on the RC connector is the camera's regular battery output. |
| 104, fig. 122 | RC connector: 6 V/BW, Ground, S1, S2. | Identifies the signals without specifying electrical input limits. The order in the drawing is not a universal pin numbering when viewed from the front of the plug. |
| 106 | S1 activates at approximately 0.5 mm of button travel; S2 at approximately 2 mm. | Supports activating S1 before S2, but does not specify a minimum interval in milliseconds. |
| 114, C-2 / fig. 131 | 3–7 mA with S1 on; 16–21 mA during release with a shutter speed of 1 or 4 seconds. | Measured in the battery supply using a dummy battery. Not measured through the RC contacts and not a specified limit for short current spikes. |
| 116, fig. 132-A | Disconnected moving-coil unit: approximately 6 Ω. | Applies to the actuator, not the resistance of the RC input. It cannot be used to calculate contact current without the intermediate circuit. |

## Indirect evidence from original accessories

[Mamiya Electromagnetic Cable Release](https://ianbfoto.com/downloads/Mamiya%20645/Mamiya%20Electromagnetic%20Cable%20Release.pdf), one page, visually inspected: Type B has approximately 4 m of straight cable. The button has a half-press for metering and a full press for release. Wire gauge, contact resistance, and input limits are not specified. Wires and connectors have finite resistance, so the camera does not require an ideal 0 Ω switch. This still does not establish a numerical resistance limit.

[Mamiya RS401](https://ianbfoto.com/downloads/Mamiya%20645/Mamiya%20645%20RZ67%20Remote%20Control%20RS401.pdf), four pages, visually inspected: describes the original wireless remote release for the RZ67 series and other cameras. No S1/S2 current, contact resistance, or threshold voltage was found in this manual.

[Jan Griffioen's repair](https://jantecnl.synology.me/en/mamiya-rz67-repair-shutter-release-electronics-problem/) states that he had the service manual but had to work from block and wiring diagrams without a functional electronic schematic. This describes the sources available to him, not proof that such a schematic does not exist.

## Component comparison

- [Omron G6K](https://components.omron.com/us-en/system/files/2026-05/datasheet_pdf/K106-E1.pdf): maximum initial contact resistance of 100 mΩ (0.1 Ω), measured at 10 mA / 1 V. The coil resistance is not the resistance seen by the camera.
- [Toshiba TLP172GM](https://toshiba.semicon-storage.com/info/TLP172GM_datasheet_en_20230525.pdf?did=36716&prodName=TLP172GM): maximum R_on of 35 Ω for a short measurement under 1 s and 50 Ω continuously, under the datasheet test conditions at 25 °C and 5 mA LED current.
- [Toshiba TLP172AM](https://toshiba.semicon-storage.com/info/docget.jsp?did=36714&prodName=TLP172AM): maximum R_on of 2 Ω under the datasheet test conditions at 25 °C and 5 mA LED current. The output is rated for 60 V / 500 mA; the limits must be assessed with derating. This is not a guarantee of camera compatibility.

Illustrative calculation, not measured contact current: If 7 mA flowed through the switch, 50 Ω would produce a 350 mV voltage drop and 2 Ω would produce 14 mV. At 21 mA, the values would be 1.05 V and 42 mV. It has not been established that the camera's battery current flows through S1 or S2. These figures are therefore not a worst-case calculation for the camera.

## Scoped next test

An external adapter connected at the already verified RC pinout can be used to compare the working mechanical relay with known series resistances in each signal wire, both individually and together. Do not use the BW pin or apply an external voltage. Check half-press, release, bulb mode, and operation with a weaker battery. This tests the contact resistance tolerance of the specific camera, but does not replace checks of PhotoMOS leakage, current spikes, switching time, or testing with other camera models.
