# This directory is NO LONGER BUILT

Nothing includes `<BNO055.h>` and no build script passes
`--library vendor/BNO055`. The code here is inert.

It is still on disk **on purpose**, as the revert path for a change that has not
yet run on hardware. Deleting it is a separate step — see the bottom of this
file.

## Why it went

Two reasons, and the second is the one that actually mattered day to day.

**Licence.** `BNO055.c` declares GPLv3-or-later. This repository is MIT, and the
directory never carried the full GPL text the header refers to. Shipping the two
together is a question that should not have to be answered at all, and it was
being deferred rather than resolved.

**It was five patches deep to make one function safe to call.** The measured
usage, across the whole project:

| symbol | call sites |
|---|---|
| `bno055_init()` | 2 (one production, one validation sketch) |
| everything else | 0 |

`bno055_read_linear_accel_xyz()` and `bno055_set_operation_mode()` appeared only
in comments. About 16 000 lines were carried for seven single-byte register
reads — because the mode handling could not be used (it never waits for the
switches it commands) and the data path was never needed (the poll reads one
48-byte burst and decodes it directly).

To make that one function safe, this directory needed:

1. a `DASHCAM_BNO055_VENDORED` marker and an `#error` guard in the transport,
   because the library resolved BY NAME and a missing flag silently linked the
   user's global copy;
2. a patch to `bno055_init()` so it stopped overwriting the caller's device
   address with 0x28 — the wrong one for this board;
3. `extern "C"` guards the header shipped without;
4. two `#pragma GCC diagnostic` lines to silence the 307 warnings the file
   emitted at `--warnings all`, which otherwise buried every warning from this
   project's own code;
5. a documented list of upstream defects to route around, including a truncated
   `sw_revision_id` and an init return value that reports only its last read.

The replacement is `lib/BNO055Regs.h` (register map, mode constants, device
struct) and `bno055Identify()` in `lib/BNO055Transport.cpp`. Together they are
about ninety lines and carry none of the above. Items 2 and 5 are fixed rather
than documented: the address is a parameter, the revision is a full 16 bits, and
the return value means what it says.

## Reverting

A tag captures the tree immediately before this change:

```bash
git diff pre-bno055-driver-removal -- peripherals/mkr_zero
```

To restore that state wholesale:

```bash
git checkout pre-bno055-driver-removal -- peripherals/mkr_zero
```

To revert by hand instead, three things have to come back together: the
`#include <BNO055.h>` and marker guard in `lib/BNO055Transport.h`, the
`bno055_init()` call in `lib/BNO055Init.cpp` (plus `bno055TransportBind`), and
`--library "%BNO_DIR%"` in the eight `BuildAndUpload.cmd` files.

## Deleting it for real

Once the replacement has been confirmed on hardware — bring-up reaching
`configured` with the correct chip and revision IDs — this whole directory
should be removed. Until then it stays: an unused directory is a licence
question, but a deleted one that turns out to be needed is a bad afternoon.

**Note that keeping it does not resolve the licence issue.** The code is still
present in the repository and therefore still distributed. The compliance
question closes when the directory is gone, not when it stops being compiled.
