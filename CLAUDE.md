# openrz67-trigger

## PCB

- The nRF board is generated from `kicad/tools/design.py`. After any change to it (or the
  generators / libraries), run `kicad/tools/regen.sh` and commit everything it writes under
  `kicad/out/` (gerber/, gerber zip, BOM, pos, DRC/ERC, renders).
  The zip is tracked so the fab-ready file is always in the repo.

## Case

- The case is generated from `case/openrz67_nrf_case.py`. After any change to it, run
  `case/export.sh` and commit the STLs it writes to `case/stl/`, so the printable files are
  always in the repo.

## Docs

- READMEs describe things as they are, for humans: what, how to build, how to use. No dates,
  no "was X until Y", no rejected alternatives.
- History and reasoning go in `docs/notes/`.
  When a change has a "why", write it there, not in the README.
- The ESP32 board is gone from `main`; it lives at tag `esp32-final`.
