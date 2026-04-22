#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path


def extract_csvdbg_lines(text: str) -> tuple[str | None, list[str], str | None]:
    """
    Collect CSVDBG rows across the full log.
    A log can contain multiple CSVDBG_BEGIN/END blocks and some may have 0 samples.
    """
    header: str | None = None
    rows: list[str] = []
    last_end_line: str | None = None

    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue

        header_idx = line.find("CSVDBG_HEADER,")
        if header_idx != -1 and header is None:
            header = line[header_idx + len("CSVDBG_HEADER,") :]
            continue

        row_idx = line.find("CSVDBG,")
        if row_idx != -1:
            rows.append(line[row_idx + len("CSVDBG,") :])
            continue

        end_idx = line.find("CSVDBG_END")
        if end_idx != -1:
            last_end_line = line[end_idx:]

    return header, rows, last_end_line


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Extract CSVDBG block from noisy serial log into clean CSV."
    )
    parser.add_argument("input", type=Path, help="Raw serial log file")
    parser.add_argument("output", type=Path, help="Output CSV path")
    args = parser.parse_args()

    raw_text = args.input.read_text(encoding="utf-8", errors="ignore")
    header, rows, end_line = extract_csvdbg_lines(raw_text)

    if header is None:
        raise SystemExit("No CSVDBG_HEADER found. Verify the log contains CSVDBG output.")
    if not rows:
        raise SystemExit("No CSVDBG rows found between CSVDBG_BEGIN and CSVDBG_END.")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="\n") as f:
        f.write(header + "\n")
        for row in rows:
            f.write(row + "\n")

    print(f"Wrote {len(rows)} rows to {args.output}")
    if end_line is not None:
        print(end_line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
