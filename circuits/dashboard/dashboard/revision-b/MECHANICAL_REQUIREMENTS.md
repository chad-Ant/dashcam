# Dashboard PCB — revision B

**ARCHIVED: superseded by ../PCB_DESIGN.md and ../release/README.md. Package sizes, LED lead spacing, drills and routing rules below are historical and must not be used for the current board.**

52 × 70 mm portrait PCB, two outer copper layers, no internal layers, 1.6 mm FR-4. Components are fitted on both faces. All external connectors open towards the right when viewed from the front; they appear on the left in a rear view.

## Fixed front layout

- RGB LED centres: 6, 16, 26, 36 and 46 mm from the left edge, 29 mm from the top. The horizontal positions were approved; the row height follows the supplied sketch.
- D1–D5: provisional 5 mm round, common-anode LEDs, 1.27 mm adjacent lead pitch, 0.8 mm finished holes. User approved these pending later measurements. The physical leads run **A → R → G → B from top to bottom in the front view**. They are rotated to clear rear connector bodies. Symbol pad numbers are 1, 4, 3, 2 respectively.
- Display envelope: 51 × 28 mm, centred horizontally, lower edge aligned with the board lower edge. Relative bounds: x=0.5–51.5 mm, y=42–70 mm. The envelope is on Dwgs.User, not a routed cutout.
- QYF-8231 display connects by five wires. J1 left-to-right in the front view is **VI2C (XIAO 3.3 V), 5 V, GND, SDA, SCL**, matching the photographed module's front pad order. Its VCC uses 5 V and VI2C uses the XIAO logic supply, separate from the accessory regulator.
- Photos show overall size and pin labels but no dimensioned mounting centres. Display mounting holes are deliberately omitted; measure the actual module and establish insulated standoff clearance before defining its mechanical attachment. The current board does not directly mate to the display header.

## Assembly

Front: RGB LEDs, D6, U1/U2, C1/C2, buzzer, display wire header and resistors. Rear: XIAO, USB-A, 10-position terminal, regulator, inductor, diode and bulk capacitors. The display overlays the front lower area and needs insulation/clearance from through-hole solder joints.

| Ref | Selected part / mechanical requirement |
|---|---|
| U1, U2 | Nexperia 74HCT595DB, SSOP-16, 0.65 mm pitch |
| U4 | LM2596S-3.3, TO-263-5 |
| D7 | Vishay SS34-M3/57T, SMC / DO-214AB |
| L1 | Bourns SRR1260-330M, 33 µH |
| C3 | Panasonic EEUFR1A681, 680 µF / 10 V, 8 mm body, 3.5 mm lead pitch |
| C4 | Panasonic EEUFR1A221, 220 µF / 10 V, 6.3 mm body, 2.5 mm lead pitch |
| D1–D5 | Provisional 5 mm RGB, A-R-G-B, 1.27 mm pitch; measure actual LEDs |
| D6 | 3 mm two-lead LED, 2.54 mm pitch |
| J2 | Local corrected SparkFun USB-A SMD geometry; match an actual connector drawing |
| J3 | Phoenix Contact MPT 0,5/10-2,54, 1725737, 10 positions, 2.54 mm pitch |
| BZ1 | Polarized buzzer, 14 mm nominal body, 10 mm pin pitch; verify actual part |

J3 replaces the larger original terminal to fit the right-side connector stack. Its body projects approximately 1.4 mm past the PCB edge, with additional space needed for wires and USB plugs. No board mounting holes have been specified.

## Electrical use

J3 pin 1 is regulated 5 V input; pin 4 is ground. The XIAO is powered separately through USB-C, with its VBUS header unconnected. J2 powers the custom 3.3 V accessory, 100–200 mA typical and 500 mA transient, and is marked 3V3 ONLY. J3 pins 2/3 carry J2 D+/D−; their routing is not a controlled-impedance USB design. Confirm the actual signalling requirements before using it as a USB data link.

Critical regulator paths use 0.8–1.2 mm traces and the feedback is sensed at the output capacitor. Default signal routing uses 0.2 mm traces, 0.15 mm clearance and 0.6/0.3 mm vias, with ground copper on both faces. Target 1 oz copper. Prototype checks remain for startup, load transients, regulator temperature, LEDs, buzzer and communications.

## Files and revision handling

The editable project is dashboard.kicad_pro, with dashboard.kicad_sch and dashboard.kicad_pcb. Bundled symbols and footprints are in dashboard.kicad_sym and dashboard.pretty. Standard KiCad 10 libraries are also required. Current exports are in release; verification reports are in release/checks.

Revision A native files and its old release are archived under revision-b. They use a superseded layout. Reload files from disk in any editor window that still shows an older board.

**Physical fit remains provisional until the LED dimensions, display attachment and unspecified connector/buzzer parts are measured. Do not treat the draft fabrication files as a verified mechanical release.**
