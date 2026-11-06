"""Fetch a recording from the rig: http://<host>/lab/dump.csv -> data/<date>/.

File naming: ``data/<YYYY-MM-DD>/<seq>_<experiment>.csv`` where <seq> is the
next free 2-digit number in that day's folder (01, 02, ...) and <experiment>
comes from the ``# experiment:`` meta line of the dump (sanitized for the
filesystem). ``--note`` appends a timestamped line to ``notes.md`` in the same
folder -- one notes.md per bench session/day.
"""

from __future__ import annotations

import argparse
import re
import sys
from datetime import date, datetime
from pathlib import Path

import requests

from .loadrun import parse_meta_lines

# repo-root/data, resolved relative to this file (analysis/src/slideranalysis/)
DEFAULT_DATA_DIR = Path(__file__).resolve().parents[3] / "data"


def sanitize(name: str) -> str:
    """Experiment name -> filesystem-safe token."""
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", name).strip("_.") or "run"


def next_seq(folder: Path) -> str:
    """Next free sequence number in the folder (01..99, then 100..999).

    2 digits keep the historical naming; busy days (2026-07-17 hit 99 mid-
    campaign) spill into 3-digit numbers, which still sort lexically after
    the 2-digit block within a day folder.
    """
    taken = set()
    for p in folder.glob("[0-9][0-9]_*"):
        taken.add(int(p.name.split("_", 1)[0]))
    for p in folder.glob("[0-9][0-9][0-9]_*"):
        taken.add(int(p.name.split("_", 1)[0]))
    n = 1
    while n in taken:
        n += 1
    if n > 999:
        raise RuntimeError(f"no free sequence number left in {folder}")
    return f"{n:02d}" if n <= 99 else f"{n:03d}"


def fetch(host: str, data_dir: Path = DEFAULT_DATA_DIR, note: str | None = None,
          timeout: float = 10.0) -> Path:
    """Download /lab/dump.csv from the device and store it. Returns the path."""
    host = re.sub(r"^https?://", "", host).rstrip("/")
    url = f"http://{host}/lab/dump.csv"
    resp = requests.get(url, timeout=timeout)
    resp.raise_for_status()
    text = resp.text

    meta_lines = []
    for line in text.splitlines():
        if line.startswith("#"):
            meta_lines.append(line)
        else:
            break
    meta = parse_meta_lines(meta_lines)
    experiment = sanitize(str(meta.get("experiment", "run")))

    folder = Path(data_dir) / date.today().isoformat()
    folder.mkdir(parents=True, exist_ok=True)
    dest = folder / f"{next_seq(folder)}_{experiment}.csv"
    dest.write_text(text, encoding="utf-8")

    if note:
        with open(folder / "notes.md", "a", encoding="utf-8") as fh:
            fh.write(f"- {datetime.now():%Y-%m-%d %H:%M} `{dest.name}`: {note}\n")
    return dest


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Fetch http://<host>/lab/dump.csv into data/<date>/<seq>_<experiment>.csv"
    )
    parser.add_argument("--host", required=True, help="device host[:port]")
    parser.add_argument("--note", default=None,
                        help="appended to notes.md in the day's data folder")
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA_DIR,
                        help=f"data root (default {DEFAULT_DATA_DIR})")
    parser.add_argument("--timeout", type=float, default=10.0, help="HTTP timeout [s]")
    args = parser.parse_args(argv)

    dest = fetch(args.host, data_dir=args.data_dir, note=args.note,
                 timeout=args.timeout)
    n_lines = sum(1 for _ in open(dest, encoding="utf-8"))
    print(f"saved {dest} ({n_lines} lines)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
