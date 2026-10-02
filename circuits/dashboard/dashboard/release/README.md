# Dashboard home-etch prototype

Completed 27 September 2026. Board: 52 x 70 mm, two copper faces. This release supersedes archived Rev A/Rev B artwork.

## Printing

Print print/toner-transfer.pdf at 100% actual size on A4 with scaling disabled. Confirm both 50 mm scale bars with a ruler. The front copper page is already mirrored; the rear is not mirrored. Transfer each page toner-side against its copper face. Do not apply another mirror operation. Use the registration crosses to align the two faces.

Before etching, check the 1:1 artwork against actual components. RGB lead fit, display attachment/standoff clearance, USB-A geometry and piezo dimensions remain provisional. Display mounting holes are omitted because their locations have not been measured.

## Assembly

Follow print/assembly-guide.pdf. Fit all 30 via wires and solder both faces before components; leave clearance beneath the piezo. Fit all five insulated wire jumpers shown in the guide. These are required electrical connections. No plated holes or solder mask are assumed. Through-hole leads must be soldered to each connected copper face, allowing access around plastic bodies.

Most generic holes are 1 mm. The selected J3 terminal needs 1.1 mm holes; USB-A shell holes are 2.3114 mm. Respect actual component drawings. RGB pads are 2 mm diameter on 2.54 mm centres to preserve an insulating gap.

Supply regulated 5 V at J3 pin 1, ground at pin 4. Power XIAO separately through USB-C. J2 supplies the custom 3.3 V accessory only, rated by the intended load of 100-200 mA normal / 500 mA transient. Prototype startup, load-step and temperature checks are still required. Data wiring is not qualified as controlled-impedance USB.

## Contents and verification

- bom.csv: circuit parts and five required wire links.
- via-wire-list.csv: numbered via locations relative to the front top-left corner.
- checks/: ERC, DRC, netlist, drill map, wire-link locations and print geometry.
- SHA256SUMS.txt: hashes of packaged project and release files.
- dashboard-home-etch-project.zip: editable KiCad project, local libraries, documentation and this release, excluding working experiments and preview images.

KiCad 10.0.6 ERC: 0 violations. DRC: 0 violations, 0 unconnected items, 0 schematic parity errors under the existing rule settings. DRC connectivity includes the five fitted jumpers; it does not mean copper alone completes those nets. Ignored rule categories are recorded in the reports. Standard KiCad 10 libraries are required alongside the bundled project libraries.
