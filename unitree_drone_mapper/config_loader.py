"""config_loader.py — Load config.yaml, return a nested dict.

Usage in any module:
    from unitree_drone_mapper.config_loader import load_config
    cfg = load_config()
    port        = cfg["server"]["port"]
    max_pts     = cfg["dsm"]["DEFAULT_MAX_PTS"]
    obs_radius  = cfg["collision"]["obstacle_radius"]

Design
------
The merge strategy is two-pass:

  Pass 1 — All sections present in config.yaml are loaded as-is.
           This ensures sections like [dsm], [flight], [slam], [camera]
           that exist in the yaml but have no _DEFAULTS entry are still
           accessible via cfg["dsm"]["DEFAULT_MAX_PTS"] etc.

  Pass 2 — For sections that DO have _DEFAULTS entries, individual
           missing keys are filled from defaults. This provides a safety
           net so unit tests and dev laptops without the yaml still work
           for the sections that matter most at runtime.

Falls back to _DEFAULTS entirely if the yaml file is not found.
"""

from pathlib import Path

# ── Config file location ──────────────────────────────────────────────────────
# config.yaml lives next to this file inside unitree_drone_mapper/
_CONFIG_PATH = Path(__file__).parent / "config.yaml"


# ── Fallback defaults ─────────────────────────────────────────────────────────
# Only sections actively read by Python code need entries here.
# The yaml is the authoritative source; these are the safety net for when
# the yaml is absent (unit tests, dev laptops without the SSD mounted).

_DEFAULTS: dict = {
    "collision": {
        "ignore_radius":   0.70,
        "obstacle_radius": 2.00,
        "caution_radius":  3.50,
        "down_cone_deg":   30.0,
        "down_fov_deg":    15.0,   # field_of_view reported to PX4 for AGL sensor
        "down_max_m":      15.0,
        "down_min_points": 10,
        "publish_hz":      10,
    },
    "dsm": {
        "DEFAULT_MAX_PTS":      80_000,
        "DEFAULT_MAX_PTS_FAST": 50_000,
        "MIN_RADIUS":           0.03,
        "MAX_RADIUS":           0.50,
    },
    "mesh": {
        "cloud_cap_rate":      1500,
        "cloud_cap_max":       500_000,
        "cloud_cap_rate_fast": 800,
        "cloud_cap_max_fast":  150_000,
        "min_points_to_process": 50000,
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
    """Load config.yaml and return a merged config dict.

    Parameters
    ----------
    path : str or None
        Override config file path (useful in tests).
        Defaults to config.yaml next to this file.

    Returns
    -------
    dict containing all yaml sections.
    Sections present in _DEFAULTS have individual missing keys filled
    from defaults. Sections not in _DEFAULTS are passed through as-is
    from the yaml. If the yaml is missing entirely, returns _DEFAULTS.
    """
    cfg_path = Path(path) if path else _CONFIG_PATH

    if not cfg_path.exists():
        return {k: dict(v) for k, v in _DEFAULTS.items()}

    try:
        import yaml
        with open(cfg_path) as f:
            loaded: dict = yaml.safe_load(f) or {}

        # ── Pass 1: include every section from yaml ───────────────────────────
        # This ensures sections like [dsm], [flight], [slam], [camera] that
        # are not in _DEFAULTS are still accessible in the returned dict.
        merged: dict = {k: dict(v) for k, v in loaded.items() if isinstance(v, dict)}

        # ── Pass 2: fill missing keys from _DEFAULTS for protected sections ───
        # For each section that has a _DEFAULTS entry, any key missing from
        # the yaml gets the default value. Keys present in yaml are kept.
        for section, defaults in _DEFAULTS.items():
            if section not in merged:
                # Section entirely absent from yaml — use full defaults
                merged[section] = dict(defaults)
            else:
                # Section present — fill only the missing individual keys
                for key, value in defaults.items():
                    merged[section].setdefault(key, value)

        return merged

    except Exception as exc:
        print(f"[config_loader] Could not load {cfg_path}: {exc} — using defaults")
        return {k: dict(v) for k, v in _DEFAULTS.items()}
