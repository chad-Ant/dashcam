"""Writes c8 candidates (scratchpad only) from isp_tuning/c5_rpi100T.isp. Nothing is installed."""
import re, pathlib, hashlib
SRC = pathlib.Path("/home/jetson/drive_logs/tools/isp_tuning/c5_rpi100T.isp")
OUT = pathlib.Path(__file__).parent
src = SRC.read_text()
lines = src.splitlines(keepends=True)
v2 = [i for i, l in enumerate(lines) if l.startswith("sharpness.v2.")]
assert len(v2) == 12 and v2 == list(range(v2[0], v2[0] + 12)), v2
modes = [(m, r) for m in ("Preview", "Still", "Video") for r in range(4)]

def write(name, header, body):
    p = OUT / f"{name}.isp"; p.write_text("".join([header + "\n"] + body))
    print(p, hashlib.sha256(p.read_bytes()).hexdigest()[:16])

# c8_sh15: sharpness index tables at the weakest end (15), MaxValue raised to 15 so nothing clamps them
b = lines[:v2[0]] + ["sharpness.v2.MaxValue = 15;\n"] + \
    [f"sharpness.v2.{m}[{r}] = {{15, 15, 15, 15, 15, 15, 15}};\n" for m, r in modes] + lines[v2[-1] + 1:]
write("c8_sh15", "# c8_sh15 (Claude, scratch): c5_rpi100T + sharpness.v2 index tables all 15 (weakest set) + sharpness.v2.MaxValue = 15", b)

# c8_v5off: sharpen block disabled (NVIDIA-staff method), v2 tables left at the vendor values
b = lines[:v2[-1] + 1] + ["sharpness.v5.enable = FALSE;\n"] + lines[v2[-1] + 1:]
write("c8_v5off", "# c8_v5off (Claude, scratch): c5_rpi100T + sharpness.v5.enable = FALSE (sharpen block off); sharpness.v2 unchanged", b)

# optional c8_v5tab15: NVIDIA AR0234 NOVA-style, ISP5/6 index table at 15 (settles v5.tab vs v2 precedence)
b = lines[:v2[-1] + 1] + ["sharpness.v5.enable = TRUE;\n"] + \
    [f"sharpness.v5.tab.{m}[{r}] = {{ 15, 15, 15, 15, 15, 15, 15 }};\n" for m, r in modes] + lines[v2[-1] + 1:]
write("c8_v5tab15", "# c8_v5tab15 (Claude, scratch, optional): c5_rpi100T + sharpness.v5.tab all 15 (as NVIDIA's AR0234 NOVA tuning); sharpness.v2 unchanged", b)
