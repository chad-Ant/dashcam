# Dashboard home-etch prototype — 27 September 2026

The double-sided toner-transfer layout and print set are complete. ERC and DRC report zero violations under existing project settings, with zero unconnected items and zero schematic parity errors. Connectivity assumes all five required insulated wire jumpers are fitted. Physical fit and powered prototype testing remain outstanding. Old exports under home-etch/obsolete-revA-release are superseded.

## Confirmed construction

- Board: 52 × 70 mm, two copper faces, no internal layers.
- Surface mount: shift registers, LM2596, all resistors, all capacitors, inductor and XIAO.
- Through hole: RGB LEDs, display wire header and piezo. USB-A/terminal mechanical retention uses the selected connector geometry.
- Five RGB centres remain x=6,16,26,36,46 mm and y=29 mm from the top-left front corner.
- RGB holes now use 2.54 mm pitch, 1 mm drill and 2 mm copper pads. Their straight lead sequence is A-R-G-B top-to-bottom from the front. Leads may need forming to fit; actual LED measurements remain pending. A 2.54 mm pad diameter was not used because it would leave no insulating gap at 2.54 mm pitch.
- Display envelope stays 51 × 28 mm, centred horizontally and bottom-aligned. A five-wire interface supplies VI2C=3.3 V, VCC=5 V, GND, SDA and SCL. Mounting holes and standoffs still require actual module measurements.
- External connector openings face right from the front, left in a rear view.

## Home-etch changes

U1/U2 are now 74HCT595D in SOIC-16, 1.27 mm lead pitch. R1–R17 and C1/C2 use hand-solder 0805 footprints. C3 is Panasonic EEEFK1A681P (680 uF, 10 V, 8 mm diameter, 10.2 mm body height); C4 is EEEFK1A221P (220 uF, 10 V, 8 mm diameter, 6.2 mm body height). Both bulk capacitors are surface mount. These replacements preserve capacitance and voltage ratings but have different ripple-current and impedance specifications from the previous radial parts.

Routing uses 0.30 mm minimum tracks and clearance, with wider power paths. The 30 vias use 1 mm holes and 2 mm pads and must be implemented with wire soldered on both faces; the board has no plated holes. Install via wires before components, especially beneath the piezo, and leave clearance for solder joints. Through-hole component leads require soldering to every copper face to which they connect; plastic header bodies may need spacing for top-side access. J3 requires 1.1 mm holes and the selected USB-A shell geometry requires 2.3114 mm holes. These component-specific exceptions must match the purchased parts.

No solder mask or printed silkscreen is assumed. Assembly drawings provide reference labels. Print release/print/toner-transfer.pdf at 100 percent actual size with scaling disabled; verify both 50 mm scale bars. Front artwork is already mirrored and rear artwork is not mirrored for toner-side-against-copper transfer. Do not mirror either page again.

## Required assembly links and deliverables

Five insulated wire jumpers complete the circuit: WL1 U2.4 to R5.1; WL2 U2.12 to U1.12; WL3 U2.14 to U1.9; WL4 U2.15 to R8.1; WL5 J2.2 to J3.3. WL1-WL4 are on the front, WL5 on the rear. They use existing solder pads; the board-only jumper footprints model fitted wires, not etched copper bridges.

- release/print/assembly-guide.pdf: front/rear placement, wire links, drill map and numbered via coordinates.
- release/print/toner-transfer.pdf: two 1:1 copper artwork pages with registration marks.
- release/bom.csv and release/via-wire-list.csv: component and via lists.
- release/checks: ERC, DRC, netlist, drill geometry and wire-link records.
- Editable project: dashboard.kicad_pro, dashboard.kicad_sch, dashboard.kicad_pcb, bundled dashboard.pretty and dashboard.kicad_sym; standard KiCad 10 libraries are also required.

## Electrical assumptions retained

Regulated 5 V enters J3 pin 1; ground is pin 4. XIAO is powered separately by USB-C, with VBUS header unused. USB-A is a custom 3.3 V accessory output, 100–200 mA normal and 500 mA transient. Whether its D+/D- conductors carry actual USB or custom signals remains unconfirmed. The design is not qualified for USB controlled impedance.

Prototype validation is still needed for regulator startup/load steps, temperature, display, LED and piezo operation. Exact display mounting geometry, LED physical leads, USB-A part and buzzer dimensions remain unverified.

Sources: https://www.nexperia.com/packages/SOT109-1.html ; https://na.industrial.panasonic.com/products/capacitors/aluminum-electrolytic-capacitors/lineup/aluminum-electrolytic-capacitors-surface-mount-type/series/88994/model/89460 ; https://industrial.panasonic.com/tw/products/pt/aluminum-cap-smd/models/EEEFK1A221P
