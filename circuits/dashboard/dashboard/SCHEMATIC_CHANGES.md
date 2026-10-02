# Schematic corrections — home-etch revision, 27 September 2026

The design assumes a regulated 5 V input on J3 pin 1, with its return on J3 pin 4. The XIAO is powered separately through its onboard USB-C connector; its VBUS header pad is unused. J2 is deliberately a **custom 3.3 V accessory connector**, not a standard 5 V USB power port. The stated accessory load is 100–200 mA normally and 500 mA transiently.

## Changes

- Kept the I2C connector as J1, renamed the USB-A connector J2, and retained terminal block J3.
- Added power-source flags on the externally supplied 5 V/GND rails and the regulated 3.3 V output.
- Marked unused XIAO D1/VBUS and final shift-register cascade output QH' with no-connect markers.
- Removed the VBUS-only flag and stub; retained an explicit note that the XIAO receives power through its onboard USB-C.
- Removed inappropriate physical footprints from power symbols and flags.
- Raised C1/C2's specified voltage rating to 10 V (100 nF, 0805 hand-solder footprints).
- Replaced C3/C4 with polarized capacitor symbols and matching surface-mount footprints. Positive terminal is pad 1; pad 2 is ground.
- Selected actual power-inductor and bulk-capacitor parts, recorded their manufacturer part numbers and ratings in the schematic, and documented the custom accessory voltage/load.

## Selected power components

| Ref | Component | Manufacturer part | Footprint |
|---|---|---|---|
| C3 | 680 µF, 10 V, ±20%, aluminum electrolytic | Panasonic EEEFK1A681P | Capacitor_SMD:CP_Elec_8x10.5 |
| C4 | 220 µF, 10 V, ±20%, aluminum electrolytic | Panasonic EEEFK1A221P | Capacitor_SMD:CP_Elec_8x6.2 |
| L1 | 33 µH shielded power inductor | Bourns SRR1260-330M | Inductor_SMD:L_Bourns_SRR1260 |

C3 has a specified 600 mA ripple-current rating and 160 mΩ maximum impedance at 100 kHz. C4 has a 300 mA rating and 260 mΩ maximum impedance at 100 kHz. These are impedance specifications, not a guaranteed minimum ESR. L1 has a 3 A RMS rating, 2.8 A typical saturation current at 25% inductance drop, and 60 mΩ maximum DCR. The application remains a 500 mA peak design; the inductor rating does not establish a 3 A rating for the assembled circuit.

Sources: [C3](https://na.industrial.panasonic.com/products/capacitors/aluminum-electrolytic-capacitors/lineup/aluminum-electrolytic-capacitors-surface-mount-type/series/88994/model/89460), [C4](https://industrial.panasonic.com/tw/products/pt/aluminum-cap-smd/models/EEEFK1A221P), [L1](https://www.bourns.com/docs/product-datasheets/srr1260.pdf), [LM2596 design guidance](https://www.ti.com/lit/ds/symlink/lm2596.pdf).

## Verification and limits

- KiCad 10.0.6 ERC: zero violations under the existing project settings; rule severities/exclusions were not changed.
- Netlist export: no annotation warning after the connector rename.
- Component-to-component net memberships were unchanged by the power-component replacements and unused-VBUS cleanup.
- Selected replacement footprints exist in the installed KiCad library; the bulk capacitors use 8 mm diameter SMD can footprints.
- Rendered schematic checked for polarity, pin connections, reference names and notes.

## Revision B changes

D1–D5 now use directly mounted through-hole RGB LEDs, with physical A-R-G-B lead order remapped to the existing symbol pins. D6 uses a directly mounted 3 mm LED. J3 uses the smaller 10-position Phoenix 1725737 terminal. J1 now provides five wires: 3.3 V logic level, 5 V display supply, ground, SDA and SCL. The 5 V display connection is the only added circuit net membership; other component pin-to-pin connectivity is retained.

The PCB is now 52 × 70 mm with a fixed front LED row and lower display envelope. See PCB_DESIGN.md for the pin ordering, provisional measurements and assembly requirements. The display photographs do not define mounting-hole centres; no holes were guessed from image scaling.

ERC does not validate regulator stability or transient performance. Verify the 3.3 V output through startup and the specified 100–200 mA to 500 mA load transition on a prototype.

## Completed home-etch adaptation

U1/U2 use Nexperia 74HCT595D SOIC-16 packages with 1.27 mm lead pitch. R1-R17 and C1/C2 use 0805 hand-solder pads. The regulator, diode, inductor, bulk capacitors and XIAO are surface mount. RGB LEDs, piezo and the display wire header remain through hole. RGB pad pitch is 2.54 mm with 1 mm drills and 2 mm copper pads; actual LED lead fit remains provisional.

Circuit pin connectivity is preserved through these package changes. The finished PCB has five board-only wire-link footprints representing required insulated assembly wires, plus 30 wire-filled vias. Final DRC has zero violations, unconnected items and schematic parity errors under existing settings. See PCB_DESIGN.md and release/README.md for print orientation, assembly and physical-fit limitations.
