"""Load config/heal_wire_trace.yaml (robot names, topics, tool offset, limits, speeds)."""
from __future__ import annotations

import os
from dataclasses import fields

import yaml

from .controller import ControllerParams
from .trajectory import ProcessParams

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CONFIG = os.path.join(REPO_ROOT, "config", "heal_wire_trace.yaml")


def load_config(path: str | None = None) -> dict:
    with open(path or DEFAULT_CONFIG) as f:
        return yaml.safe_load(f)


def _filter(cls, d: dict):
    names = {f.name for f in fields(cls)}
    unknown = set(d) - names
    if unknown:
        raise KeyError(f"Unknown {cls.__name__} keys in config: {sorted(unknown)}")
    out = {}
    for k, v in d.items():
        out[k] = tuple(v) if isinstance(v, list) else v
    return cls(**out)


def process_params(cfg: dict, **overrides) -> ProcessParams:
    d = dict(cfg.get("processing", {}))
    d.update({k: v for k, v in overrides.items() if v is not None})
    return _filter(ProcessParams, d)


def controller_params(cfg: dict, **overrides) -> ControllerParams:
    d = dict(cfg.get("controller", {}))
    d.update({k: v for k, v in overrides.items() if v is not None})
    return _filter(ControllerParams, d)


def data_dir(*parts) -> str:
    p = os.path.join(REPO_ROOT, "data", *parts)
    os.makedirs(p if not os.path.splitext(p)[1] else os.path.dirname(p), exist_ok=True)
    return p
