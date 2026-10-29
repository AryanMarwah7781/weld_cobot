# Mounting reference: Lite 6 flange, RealSense D405, Arducam B0495

All dimensions in mm, taken from the manufacturers' drawings (sources at the bottom).
Items marked **verify** disagree between sources or are not on any drawing; check them on the real part before printing.

## UFACTORY Lite 6 tool flange (gripper removed)

| Feature | Value |
|---|---|
| Flange outer diameter | Ø75 |
| Bolt circle A (ISO 9409-1-50-4-M6) | 4 × M6, 10 deep, on PCD Ø50, at 45°, 135°, 225°, 315° |
| Bolt circle B | 2 × M6, 10 deep, on PCD Ø63, at 90° and 270° |
| Locating hole | 1 × Ø6 (+0.1/0), 10 deep, on PCD Ø50 at 0° (between two of the M6 holes) |
| Centre recess | Ø31.5, 6 deep, 1×45° chamfer: a centring spigot on the adapter can locate in it |
| Tool I/O (connector on the side of the flange) | 2 digital in, 2 digital out, 2 analog in, Modbus RTU |

- **Thread size: M6.** The manual's End Flange section gives 6 × M6 threaded holes plus the Ø6 locating hole, and the gripper instructions fix the gripper with M6 bolts. Its spec summary table's "DIN ISO 9409-1-A50/63 (M5*6)" contradicts both; ignore it (an M5 would sit loose in an M6 tap).
- Angles are measured from the locating hole on the drawing. In the user's photo (2026-10-20) the plain locating hole is at the top-left. Match the adapter's pin to it.
- The manual's spec table gives repeatability as ±1 mm; `claude/claude.md` says ±0.5 mm (website figure).

## Intel RealSense D405

| Feature | Value |
|---|---|
| Body | 42 × 42 × 23 (±0.15), weight 58 g nominal (±10 %) |
| Back face mounting | **2 × M3×0.5, 20.00 apart**, max screw insertion **4.0** |
| Bottom mounting | 1 × 1/4-20 UNC tripod, max insertion 5.0; centre **8.35 from the back face** (14.65 from the front glass, by subtraction). Datasheet Figure 10-13: the back view carries the logo, the front view the two lenses |
| Imagers (front) | 2 lenses, baseline **18.00**; the left imager is the depth/colour origin, 9 mm from the tripod-hole centreline |
| Depth zero point | 3.7 behind the front cover glass |
| USB connector (side face) | Datasheet: **USB 3 Micro-B**, with 2 × M2×0.4 screw-lock holes 18.00 apart, max insertion 3 (**verify on the unit**) |
| Depth FOV | 87° H × 58° V × 95° D; best range about 7–50 cm |

## Arducam B0495 (AR0234, back-cover version)

| Feature | Value |
|---|---|
| Board | 34 × 34, corner radius R2.5 |
| Mounting holes | **4, on a 28 × 28 square** (3 in from each edge); hole size not on the drawing (**verify**, likely M2) |
| Lens | M12 mount, Ø14 barrel, front of lens 20 ahead of the front PCB; lens holder 20 wide (holder screws 15.8 apart) |
| Overall depth | 40.2 (lens front to back of the stack) |
| USB | USB-C on the rear board, **cable exits backwards** along the optical axis |
| Lens optics | 3.6 mm EFL, F3.0, FOV 82° H × 66° V × 95° D (a newer listing says 3.56 mm, F2.5, 85° × 69° × 98°), manual focus, factory focus 2 m–∞ |
| Weight | not listed (**weigh it**) |
| Power | 1.45 W at 1920×1200@50 |

## Sources
- UFACTORY Lite 6 User Manual, §1.2.2.5 "End-effector Installation" (drawing) and Appendix 2 "Technical Specifications": https://static.generation-robots.com/media/ufactory-lite6-user-manual.pdf, also https://docs.lite6.ufactory.cc/8.technical_specifications.html
- Intel RealSense D400 Series Datasheet, rev 337029-017 (Oct 2024): Table 3-54 (D405 dimensions), §3.7.6 (D405 USB Micro-B), Table 4-16 (depth start point), Figure 4-8 and Table 4-20 (D405 drawing): https://realsenseai.com/wp-content/uploads/2024/10/Intel-RealSense-D400-Series-Datasheet-October-2024.pdf
- Arducam B0495 datasheet V1.0 (2024-01-20), §3–4: https://blog.arducam.com/downloads/datasheet/B0495_AR0234_Global_Shutter_USB3.0_Camera_Module_Datasheet.pdf
