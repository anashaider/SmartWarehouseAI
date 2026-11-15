"""3D rendering benchmark (used for the before/after report).

Measures, on a real on-screen window with vsync disabled:
  * frame time (mean / p50 / p95) and FPS while the simulation runs,
  * CPU time of the Python per-frame update (WarehouseApp3D._update),
  * process memory (Windows working set / private bytes),
  * scene size (GeomNodes, Geoms, vertices) under render,
  * cost and memory growth of repeated layout rebuilds (leak check).

Usage:  python tools/bench3d.py --label before --quality medium --camera overview
Results are appended to data/results/bench3d.jsonl (git-ignored).
"""

from __future__ import annotations

import argparse
import ctypes
import gc
import json
import statistics
import sys
import time
from ctypes import wintypes
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


class _PMC(ctypes.Structure):
    _fields_ = [("cb", wintypes.DWORD), ("PageFaultCount", wintypes.DWORD),
                ("PeakWorkingSetSize", ctypes.c_size_t), ("WorkingSetSize", ctypes.c_size_t),
                ("QuotaPeakPagedPoolUsage", ctypes.c_size_t), ("QuotaPagedPoolUsage", ctypes.c_size_t),
                ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t), ("QuotaNonPagedPoolUsage", ctypes.c_size_t),
                ("PagefileUsage", ctypes.c_size_t), ("PeakPagefileUsage", ctypes.c_size_t),
                ("PrivateUsage", ctypes.c_size_t)]


def memory_mb() -> dict[str, float]:
    if not sys.platform.startswith("win"):
        return {}
    pmc = _PMC()
    pmc.cb = ctypes.sizeof(_PMC)
    k32 = ctypes.WinDLL("kernel32", use_last_error=True)
    psapi = ctypes.WinDLL("psapi", use_last_error=True)
    k32.GetCurrentProcess.restype = wintypes.HANDLE
    psapi.GetProcessMemoryInfo.argtypes = [wintypes.HANDLE, ctypes.POINTER(_PMC), wintypes.DWORD]
    psapi.GetProcessMemoryInfo.restype = wintypes.BOOL
    psapi.GetProcessMemoryInfo(k32.GetCurrentProcess(), ctypes.byref(pmc), pmc.cb)
    return {"working_set_mb": round(pmc.WorkingSetSize / 2**20, 1), "private_mb": round(pmc.PrivateUsage / 2**20, 1)}


def scene_stats(render) -> dict[str, int]:
    geom_nodes = render.findAllMatches("**/+GeomNode")
    geoms = tris = 0
    for np_ in geom_nodes:
        gn = np_.node()
        for i in range(gn.getNumGeoms()):
            geoms += 1
            g = gn.getGeom(i)
            for k in range(g.getNumPrimitives()):
                tris += g.getPrimitive(k).getNumFaces()
    return {"geom_nodes": geom_nodes.getNumPaths(), "geoms": geoms, "triangles": tris,
            "nodes_total": render.findAllMatches("**").getNumPaths()}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", required=True)
    ap.add_argument("--quality", default="medium")
    ap.add_argument("--layout", default="large")
    ap.add_argument("--camera", default="overview")
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--rebuilds", type=int, default=6)
    ap.add_argument("--paused", action="store_true", help="measure a paused (static) scene")
    args = ap.parse_args()

    from interface3d import app3d
    original = app3d.configure_panda

    def no_vsync(offscreen, size, title="bench"):
        original(offscreen, size, title)
        from panda3d.core import loadPrcFileData
        loadPrcFileData("", "sync-video false")

    app3d.configure_panda = no_vsync
    from config.settings import SimulationConfig

    mem0 = memory_mb()
    t0 = time.perf_counter()
    app = app3d.WarehouseApp3D(SimulationConfig(layout=args.layout, agent_type="utility_based"),
                               quality=args.quality, intro=False, size=(1600, 900))
    startup_s = time.perf_counter() - t0
    update_times: list[float] = []
    inner = app._update

    def timed_update(dt):
        s = time.perf_counter()
        inner(dt)
        update_times.append(time.perf_counter() - s)

    app._update = timed_update
    app.cmd_camera(args.camera)
    app.cmd_speed(4)                     # 8 ticks/s
    if args.paused:
        for _ in range(12):
            app.ctl.step_once()
    else:
        app.ctl.start()
    tm = app.base.taskMgr
    warm = time.perf_counter() + 2.0
    while time.perf_counter() < warm:
        tm.step()
    update_times.clear()
    frame_times: list[float] = []
    end = time.perf_counter() + args.seconds
    last = time.perf_counter()
    while time.perf_counter() < end:
        tm.step()
        now = time.perf_counter()
        frame_times.append(now - last)
        last = now
        if app.ctl.engine.done and not args.paused:   # keep the robots busy for the whole measurement
            app.cmd_reset()
            app.ctl.start()
    mem_run = memory_mb()
    stats = scene_stats(app.base.render)

    # Rebuild / leak check: switch layouts repeatedly, return to the original.
    gc.collect()
    nodes_before = app.base.render.findAllMatches("**").getNumPaths()
    rb_times = []
    for i in range(args.rebuilds):
        s = time.perf_counter()
        app.cmd_config(layout=("small", "medium", args.layout)[i % 3])
        rb_times.append(time.perf_counter() - s)
        for _ in range(5):
            tm.step()
    app.cmd_config(layout=args.layout)
    for _ in range(5):
        tm.step()
    gc.collect()
    nodes_after = app.base.render.findAllMatches("**").getNumPaths()
    mem_after = memory_mb()

    ft = sorted(frame_times)
    result = {
        "label": args.label, "paused": args.paused, "quality": args.quality, "layout": args.layout, "camera": args.camera,
        "gpu": app.base.win.getGsg().getDriverRenderer(),
        "frames": len(ft),
        "fps_mean": round(len(ft) / sum(ft), 1),
        "frame_ms_mean": round(1000 * statistics.mean(ft), 2),
        "frame_ms_p50": round(1000 * ft[len(ft) // 2], 2),
        "frame_ms_p95": round(1000 * ft[int(len(ft) * 0.95)], 2),
        "update_cpu_ms_mean": round(1000 * statistics.mean(update_times), 3),
        "update_cpu_ms_p95": round(1000 * sorted(update_times)[int(len(update_times) * 0.95)], 3),
        "startup_s": round(startup_s, 2),
        "rebuild_s_mean": round(statistics.mean(rb_times), 3),
        "render_nodes_before_rebuilds": nodes_before,
        "render_nodes_after_rebuilds": nodes_after,
        "mem_start": mem0, "mem_running": mem_run, "mem_after_rebuilds": mem_after,
        **stats,
    }
    out = ROOT / "data" / "results" / "bench3d.jsonl"
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("a", encoding="utf-8") as f:
        f.write(json.dumps(result) + "\n")
    print(json.dumps(result))
    app.destroy()


if __name__ == "__main__":
    main()
