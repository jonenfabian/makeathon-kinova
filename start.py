"""Python-Start für die Kinova-Simulation, ohne App-Installer."""
from __future__ import annotations

import runpy
import sys
from pathlib import Path

MINDEST_PYTHON = (3, 10)


def python_pruefen(version=sys.version_info) -> str | None:
    """Verständliche Meldung, wenn das Python zu alt ist (z. B. Apples 3.9 am Mac)."""
    if tuple(version[:2]) >= MINDEST_PYTHON:
        return None
    return (f"Python {version[0]}.{version[1]} ist zu alt, nötig ist mindestens 3.10 (empfohlen 3.11). "
            "Am Mac ist `python3` ohne weitere Installation oft Apples Python 3.9. Installiert Python 3.11 "
            "(python.org oder Homebrew), legt `.venv` damit neu an und installiert requirements.txt erneut.")


def main():
    meldung = python_pruefen()
    if meldung:
        print(meldung, file=sys.stderr)
        return 2
    root = Path(__file__).resolve().parent
    extra = sys.argv[1:]
    if extra and extra[0] == "kinova":  # Aufruf aus den Startkarten und dem früheren gemeinsamen Repository
        extra.pop(0)
    sys.path.insert(0, str(root / "kinova/simulation"))
    sys.argv = [str(root / "kinova/simulation/server.py"), "--sim", *extra]
    runpy.run_path(sys.argv[0], run_name="__main__")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
