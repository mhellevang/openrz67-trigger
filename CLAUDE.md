# openrz67-trigger

## PCB

- After any change to `esp32/pcb/kicad/*.kicad_pcb` or `*.kicad_sch`, run `esp32/pcb/kicad/tools/regen.sh`
  and commit everything it writes under `esp32/pcb/kicad/out/` (gerber/, gerber zip, BOM, pos, DRC/ERC, renders).
  The zip is tracked so the fab-ready file is always in the repo.

## Docs

- READMEs describe things as they are, for humans: what, how to build, how to use. No dates,
  no "was X until Y", no rejected alternatives.
- History and reasoning go in `esp32/pcb/kicad/notes/revisions.md` and `esp32/case/notes/design-history.md` for the ESP32 board,
  `nrf/notes/` for the nRF board.
  When a change has a "why", write it there, not in the README.
