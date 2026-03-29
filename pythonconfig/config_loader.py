"""config_loader.py — Load dronepi.yaml, return a nested dict.

Usage in any module:
    from config.config_loader import load_config
    cfg = load_config()
    port = cfg["server"]["port"]

Falls back to built-in defaults if the YAML file is not found
(e.g. during unit tests or on a dev laptop without the config mounted).
"""

import os
from pathlib import Path

_CONFIG_PATH = Path(__file__).parent / "dronepi.yaml"

_DEFAULTS = {
    "collision": {
        "ignore_radius":   0.70,
        "obstacle_radius": 2.00,
        "caution_radius":  3.50,
        "down_cone_deg":   30.0,
        "down_max_m":      15.0,
        "down_min_points": 10,
        "publish_hz":      10,
    },
    "mesh": {
        "cloud_cap_rate":      1500,
        "cloud_cap_max":       500_000,
        "cloud_cap_rate_fast": 800,
        "cloud_cap_max_fast":  150_000,
    },
    "server": {
        "port":      8080,
        "bind_addr": "0.0.0.0",
    },
    "paths": {
        "maps_dir":    "/mnt/ssd/maps",
        "rosbags_dir": "/mnt/ssd/rosbags",
    },
}


def load_config(path: str | None = None) -> dict:
    """Load dronepi.yaml and return merged config dict.

    Parameters
    ----------
    path : str or None
        Override config file path (useful in tests).
        Defaults to config/dronepi.yaml next to this file.

    Returns
    -------
    dict with keys: collision, mesh, server, paths
    Always returns a valid dict even if the file is missing.
    """
    cfg_path = Path(path) if path else _CONFIG_PATH

    if not cfg_path.exists():
        return _DEFAULTS.copy()

    try:
        import yaml
        with open(cfg_path) as f:
            loaded = yaml.safe_load(f) or {}
        # Merge: loaded values override defaults, missing keys fall back
        merged = {}
        for section, defaults in _DEFAULTS.items():
            merged[section] = {**defaults, **loaded.get(section, {})}
        return merged
    except Exception as e:
        print(f"[config_loader] Could not load {cfg_path}: {e} — using defaults")
        return _DEFAULTS.copy()
