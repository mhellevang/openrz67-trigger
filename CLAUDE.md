# openrz67-trigger

## PCB

- The nRF board is generated from `nrf/kicad/tools/design.py`. After any change to it (or the
  generators / libraries), run `nrf/kicad/tools/regen.sh` and commit everything it writes under
  `nrf/kicad/out/` (gerber/, gerber zip, BOM, pos, DRC/ERC, renders).
  The zip is tracked so the fab-ready file is always in the repo.

## Docs

- READMEs describe things as they are, for humans: what, how to build, how to use. No dates,
  no "was X until Y", no rejected alternatives.
- History and reasoning go in `nrf/notes/`.
  When a change has a "why", write it there, not in the README.
- The ESP32 board is gone from `main`; it lives at tag `esp32-final`.
