#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Motor Profile Loader
=====================
Loads motor configuration YAML profiles from the motor_profiles/ directory.

Usage:
    from motor_profiles import load_profile
    profile = load_profile("seat")   # -> dict
    print(profile["electrical"]["pole_pairs"])
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, List

import yaml


_PROFILES_DIR = Path(__file__).parent


def list_profiles() -> List[str]:
    """Return names of available motor profiles (without .yaml extension)."""
    return sorted(
        p.stem for p in _PROFILES_DIR.glob("*.yaml")
    )


def load_profile(name: str) -> Dict[str, Any]:
    """
    Load a motor profile by name.

    Args:
        name: Profile name (e.g. "seat", "fan"). Looks for ``<name>.yaml``
              in the motor_profiles/ directory.

    Returns:
        Parsed YAML as a nested dict.

    Raises:
        FileNotFoundError: If profile YAML does not exist.
    """
    path = _PROFILES_DIR / f"{name}.yaml"
    if not path.exists():
        available = list_profiles()
        raise FileNotFoundError(
            f"Motor profile '{name}' not found at {path}.\n"
            f"Available profiles: {available}"
        )
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def get_test_param(profile: Dict[str, Any], key: str, default: Any = None) -> Any:
    """Get a value from the profile's test section with a fallback default."""
    return profile.get("test", {}).get(key, default)


def get_speed_step_pairs(profile: Dict[str, Any]) -> List[tuple]:
    """Get speed step test pairs as list of (from_rpm, to_rpm) tuples."""
    pairs = get_test_param(profile, "speed_step_pairs", [])
    return [tuple(p) for p in pairs]


if __name__ == "__main__":
    # Quick self-test
    print(f"Available profiles: {list_profiles()}")
    for name in list_profiles():
        p = load_profile(name)
        print(f"\n── {p['motor_name']} ({name}) ──")
        print(f"  Pole pairs:    {p['electrical']['pole_pairs']}")
        print(f"  Rs = {p['electrical']['rs_ohm']} Ω")
        print(f"  Ls = {p['electrical']['ls_h']} H")
        print(f"  λ  = {p['electrical']['flux_linkage_vs']} Vs")
        print(f"  Max RPM:       {p['limits']['max_rpm']}")
        print(f"  Speed steps:   {get_speed_step_pairs(p)}")
