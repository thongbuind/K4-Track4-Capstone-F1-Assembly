"""Run reproducible simulation experiments and retain unedited output logs."""
import json
from datetime import datetime
import platform
from pathlib import Path
import subprocess
import sys
import time
from zoneinfo import ZoneInfo
from importlib.metadata import version

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "evidence"
CASES = {
    "pick_baseline": ["demo_pick", "--seconds", "60", "--seed", "0"],
    "pick_slip_20pct": ["demo_pick", "--seconds", "60", "--seed", "0", "--slip", "0.20"],
    "assembly_baseline": ["main", "--seconds", "120", "--seed", "0"],
}


def main():
    OUT.mkdir(exist_ok=True)
    metadata = {"started_at": datetime.now(ZoneInfo("Asia/Ho_Chi_Minh")).isoformat(),
                "python": sys.version, "platform": platform.platform(),
                "pybullet": version("pybullet"), "numpy": version("numpy"), "runs": []}
    for name, (module, *args) in CASES.items():
        cmd = [sys.executable, "-m", f"f1_assembly.{module}", "--direct", *args]
        start = time.perf_counter()
        result = subprocess.run(cmd, cwd=ROOT, text=True, capture_output=True)
        (OUT / f"{name}.txt").write_text(result.stdout + result.stderr, encoding="utf-8")
        metadata["runs"].append({"case": name, "command": cmd[1:],
                                 "returncode": result.returncode,
                                 "wall_seconds": round(time.perf_counter() - start, 3)})
        (OUT / "environment.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        print(f"{name}: exit={result.returncode}", flush=True)
        print(result.stdout[-1800:], flush=True)
        if result.returncode:
            print(result.stderr, file=sys.stderr)
            raise SystemExit(result.returncode)


if __name__ == "__main__":
    main()
