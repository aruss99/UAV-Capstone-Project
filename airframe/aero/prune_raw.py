"""Drop VSPAero's bulky scratch output, keeping everything results trace to.

A single 357-point sweep leaves a ~100 MB .adb solution database and one ~46 KB
per-case field file, none of which any table depends on.  This directory sits
inside OneDrive, so leaving roughly a gigabyte of solver scratch here means
syncing it.

KEPT (this is the traceability set -- every number in tables/ comes from these):
    case.vspscript      the exact script that ran
    *.vspaero           the setup file VSPAero actually consumed
    *.polar *.stab      integrated forces and moments
    *.lod               spanwise loading, used for the validated band
    *.history           convergence
    *.csf *.vkey        control-surface and tag definitions
    run.log             solver stdout
    *ParasiteBuildUp.csv

Run with --dry-run to see what would go.
"""

from __future__ import annotations

import sys
from pathlib import Path

RAW = Path(__file__).resolve().parent / "raw"

KEEP_SUFFIX = {".vspscript", ".vspaero", ".polar", ".stab", ".lod", ".history",
               ".csf", ".vkey", ".log", ".taglist"}
KEEP_NAME_PARTS = ("ParasiteBuildUp", "CompGeom")


def keep(p: Path) -> bool:
    if p.suffix in KEEP_SUFFIX:
        return True
    return any(part in p.name for part in KEEP_NAME_PARTS)


def main() -> None:
    dry = "--dry-run" in sys.argv
    freed = 0
    removed = 0
    for p in sorted(RAW.rglob("*")):
        if not p.is_file() or keep(p):
            continue
        size = p.stat().st_size
        freed += size
        removed += 1
        if dry:
            print(f"  would remove {p.relative_to(RAW)} ({size/1e6:.1f} MB)")
        else:
            try:
                p.unlink()
            except OSError as e:
                print(f"  could not remove {p.relative_to(RAW)}: {e}")
                freed -= size
                removed -= 1
    verb = "would free" if dry else "freed"
    print(f"{verb} {freed/1e6:.1f} MB across {removed} files")


if __name__ == "__main__":
    main()
