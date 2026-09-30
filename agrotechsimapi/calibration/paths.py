"""Writable data paths; installed packages are always treated as read-only."""
import os
import sys
from pathlib import Path


def data_dir() -> Path:
    override = os.environ.get("AGROTECHSIMAPI_DATA_DIR")
    if override:
        return Path(override).expanduser().resolve()
    if sys.platform == "win32":
        base = Path(os.environ.get("LOCALAPPDATA", Path.home() / "AppData/Local"))
    elif sys.platform == "darwin":
        base = Path.home() / "Library/Application Support"
    else:
        configured = os.environ.get("XDG_DATA_HOME", "")
        base = Path(configured) if configured and Path(configured).is_absolute() else Path.home() / ".local/share"
    return base / "agrotechsimapi"
