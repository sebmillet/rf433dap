RF433dap - RF433 Decode Any Protocol
====================================

Uses a RF433Mhz component plugged on an Arduino to listen to signals and decode
it.

The decoder tries to be as flexible as possible to decode any protocols,
without pre-knowledge about encoding. To be generic enough, only the
**relationships** between timings is analyzed, no pre-defined timing is used.

Assumes one of the following auto-synchronization protocols:
- Tri-bit
- Manchester
- Raw (if none of tri-bit and Manchester works), meaning, no
  auto-synchonization.

Schematic: Arduino plugged on a RF433 RX component, like MX-RM-5V.
The RF digital pin must be plugged on Arduino pin D2.

