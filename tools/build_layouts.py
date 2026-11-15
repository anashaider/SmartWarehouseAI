"""Regenerate the predefined layout JSON files in data/layouts/.

The maps are hand-designed; this script only checks their shape and writes
them in the documented JSON format. Run: python tools/build_layouts.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from environment.scenario_generator import scenario_from_dict  # noqa: E402

SMALL = [
    "R.....R.....R.",
    "..............",
    ".##P.##..##...",
    ".##..##.P##.P.",
    "..............",
    ".##P.##..##...",
    ".##..##P.##P..",
    "..............",
    "...D......D...",
]

MEDIUM = [
    "R........R........R.",
    "....................",
    ".###.###P.###.###...",
    ".###P###..###.###P..",
    "....................",
    ".###.###..###P###...",
    ".###.###.P###.###.P.",
    "....P...............",
    ".###.###..###.###...",
    ".###P###..###.###P..",
    "..........P.........",
    "....................",
    "..D......D......D...",
]


def large() -> list[str]:
    w, h = 28, 18
    g = [["."] * w for _ in range(h)]
    for r0 in (2, 5, 8, 11, 14):
        for c0, c1 in ((2, 6), (8, 12), (15, 19), (21, 25)):
            for r in (r0, r0 + 1):
                for c in range(c0, c1 + 1):
                    g[r][c] = "#"
    # Irregularities: a blocked cross-aisle and two pallets.
    for c in (7, 20):
        g[8][c] = g[9][c] = "#"
    g[4][14] = "#"
    g[10][1] = "#"
    for r, c in ((0, 0), (0, 13), (0, 27)):
        g[r][c] = "R"
    for r, c in ((17, 4), (17, 11), (17, 17), (17, 24)):
        g[r][c] = "D"
    parcels = [(4, 3), (4, 22), (7, 10), (7, 17), (10, 5), (10, 23), (13, 9),
               (13, 18), (16, 3), (16, 22), (1, 9), (1, 16), (10, 13), (7, 26)]
    for r, c in parcels:
        assert g[r][c] == ".", (r, c)
        g[r][c] = "P"
    return ["".join(row) for row in g]


LAYOUTS = {
    "small": dict(name="Small Warehouse", seed=11, size_class="small",
                  description="14x9 floor, 2 stations, 6 parcels.", map=SMALL),
    "medium": dict(name="Medium Warehouse", seed=22, size_class="medium",
                   description="20x13 floor, 3 stations, 10 parcels.", map=MEDIUM),
    "large": dict(name="Large Warehouse", seed=33, size_class="large",
                  description="28x18 floor, 4 stations, 14 parcels, blocked cross-aisles.",
                  map=large()),
}

if __name__ == "__main__":
    out_dir = ROOT / "data" / "layouts"
    out_dir.mkdir(parents=True, exist_ok=True)
    for key, data in LAYOUTS.items():
        data = {"id": key, **data,
                "legend": {".": "floor", "#": "shelf", "D": "delivery station",
                           "R": "robot start", "P": "parcel pickup"}}
        sc = scenario_from_dict(data, scenario_id=key)  # validates
        (out_dir / f"{key}.json").write_text(json.dumps(data, indent=2), encoding="utf-8")
        print(f"{key}: {sc.width}x{sc.height}, {len(sc.parcels)} parcels, "
              f"{len(sc.stations)} stations")
        print("\n".join(data["map"]))
