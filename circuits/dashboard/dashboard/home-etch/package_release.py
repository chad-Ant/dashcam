from pathlib import Path
import hashlib, json, zipfile

root = Path(__file__).resolve().parent.parent
release = root / 'release'
drc = json.loads((release / 'checks/pcb-drc.json').read_text())
erc = json.loads((release / 'checks/schematic-erc.json').read_text())
assert not drc['violations'] and not drc['unconnected_items'] and not drc['schematic_parity']
assert all(not sheet['violations'] for sheet in erc['sheets'])
files = [root / name for name in (
    'dashboard.kicad_pro', 'dashboard.kicad_sch', 'dashboard.kicad_pcb',
    'dashboard.kicad_sym', 'fp-lib-table', 'sym-lib-table',
    'PCB_DESIGN.md', 'SCHEMATIC_CHANGES.md')]
files += sorted((root / 'dashboard.pretty').glob('*.kicad_mod'))
files += [release / 'README.md', release / 'bom.csv', release / 'via-wire-list.csv']
files += sorted((release / 'print').glob('*.pdf'))
files += sorted(p for p in (release / 'checks').iterdir() if p.is_file())
assert all(p.is_file() for p in files)
manifest = release / 'SHA256SUMS.txt'
manifest.write_text(''.join(f'{hashlib.sha256(p.read_bytes()).hexdigest()}  {p.relative_to(root).as_posix()}\n' for p in files), encoding='utf-8')
files.append(manifest)
archive = release / 'dashboard-home-etch-project.zip'
with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as z:
    for p in files:
        z.write(p, p.relative_to(root).as_posix())
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    assert len(z.namelist()) == len(files)
print(json.dumps({'archive': str(archive), 'files': len(files), 'bytes': archive.stat().st_size,
                  'erc_violations': 0, 'drc_violations': 0, 'unconnected': 0, 'parity_errors': 0}))
