#!/usr/bin/env python3
"""Convert a SAX netlist into a gdsfactory YAML netlist.

SAX netlist (dict, or JSON/YAML file):
    {
      "instances": {"lft": "mmi1x2",                       # str form
                    "top": {"component": "straight",      # dict form
                            "settings": {"length": 10}}},
      "connections": {"lft,out0": "top,in0", ...},
      "ports": {"in0": "lft,in", "out0": "rgt,out"},
    }

gdsfactory YAML:
    instances: {...}
    placements: {...}
    routes: {optical: {links: {"a,o1": "b,o2"}, routing_strategy: route_bundle}}
    ports: {...}

Usage:
    python sax2gf_yaml.py netlist.json -o netlist.pic.yml
    python sax2gf_yaml.py netlist.yml --mode nets --dx 150
"""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any

import yaml

# Default SAX -> gdsfactory port-name map (extend/override with --port-map).
DEFAULT_PORT_MAP = {
    "in": "o1", "out": "o2",
    "in0": "o1", "in1": "o2", "out0": "o3", "out1": "o4",
}


def _split(ref: str) -> tuple[str, str]:
    """'inst,port' -> ('inst', 'port'). SAX uses a comma separator."""
    inst, _, port = ref.partition(",")
    if not port:
        raise ValueError(f"Bad port reference {ref!r}; expected 'instance,port'")
    return inst.strip(), port.strip()


def _port_name(port: str, port_map: dict[str, str], auto: bool) -> str:
    if port in port_map:
        return port_map[port]
    if auto:
        # in0/out1/... style -> o<N> ; leave already-gf names (o1, e1) alone
        m = re.fullmatch(r"(?:in|out|p|port)?(\d+)", port)
        if m:
            return f"o{int(m.group(1)) + 1}"
    return port


def _ref(ref: str, port_map: dict[str, str], auto: bool) -> str:
    inst, port = _split(ref)
    return f"{inst},{_port_name(port, port_map, auto)}"


def _convert_instance(spec: Any) -> dict[str, Any]:
    if isinstance(spec, str):
        return {"component": spec}
    if isinstance(spec, dict):
        out = {"component": spec.get("component") or spec.get("model")}
        if out["component"] is None:
            raise ValueError(f"Instance has no 'component': {spec}")
        settings = spec.get("settings")
        if settings:
            out["settings"] = dict(settings)
        return out
    raise TypeError(f"Unsupported instance spec: {spec!r}")


def sax_to_gdsfactory(
    netlist: dict[str, Any],
    mode: str = "routes",
    port_map: dict[str, str] | None = None,
    auto_port_names: bool = True,
    dx: float = 200.0,
    routing_strategy: str = "route_bundle",
    cross_section: str | None = None,
    name: str | None = None,
) -> dict[str, Any]:
    """Return a gdsfactory-compatible netlist dict.

    mode:
      'routes'      -> SAX connections become routing links (default; logical
                       connections generally need waveguides between placed
                       instances).
      'connections' -> SAX connections become abutting gdsfactory 'connections'
                       (ports snap together; no routing).
      'nets'        -> schematic-style 'nets' list (gdsfactory schematic format).
    """
    pm = {**DEFAULT_PORT_MAP, **(port_map or {})}
    conv = lambda r: _ref(r, pm, auto_port_names)  # noqa: E731

    instances = {
        iname: _convert_instance(spec)
        for iname, spec in netlist.get("instances", {}).items()
    }

    # Simple left-to-right placement following connection order (BFS from sources).
    placements = _auto_place(instances, netlist.get("connections", {}), dx)

    out: dict[str, Any] = {}
    if name:
        out["name"] = name
    out["instances"] = instances
    out["placements"] = placements

    connections = {conv(a): conv(b) for a, b in netlist.get("connections", {}).items()}

    if mode == "routes":
        route: dict[str, Any] = {
            "links": connections,
            "routing_strategy": routing_strategy,
        }
        if cross_section:
            route["settings"] = {"cross_section": cross_section}
        out["routes"] = {"optical": route}
    elif mode == "connections":
        out["connections"] = connections
    elif mode == "nets":
        out["nets"] = [[a, b] for a, b in connections.items()]
    else:
        raise ValueError(f"Unknown mode {mode!r}")

    ports = {conv(p) if False else _port_name(p, pm, auto_port_names): conv(ref)
             for p, ref in netlist.get("ports", {}).items()}
    if ports:
        out["ports"] = ports
    return out


def _auto_place(instances, connections, dx):
    """Column-based placement: each instance's x = depth along connections * dx."""
    names = list(instances)
    succ: dict[str, set[str]] = {n: set() for n in names}
    indeg = {n: 0 for n in names}
    for a, b in connections.items():
        ia, ib = _split(a)[0], _split(b)[0]
        if ia in succ and ib in succ and ib not in succ[ia] and ia != ib:
            succ[ia].add(ib)
            indeg[ib] += 1
    depth = {n: 0 for n in names}
    queue = [n for n in names if indeg[n] == 0] or names[:1]
    seen = set()
    while queue:
        n = queue.pop(0)
        if n in seen:
            continue
        seen.add(n)
        for m in succ[n]:
            depth[m] = max(depth[m], depth[n] + 1)
            indeg[m] -= 1
            if indeg[m] <= 0:
                queue.append(m)
    rows: dict[int, int] = {}
    placements = {}
    for n in names:
        d = depth[n]
        r = rows.get(d, 0)
        rows[d] = r + 1
        placements[n] = {"x": float(d * dx), "y": float(r * dx / 2)}
    return placements


def _load(path: Path) -> dict[str, Any]:
    text = path.read_text()
    return json.loads(text) if path.suffix == ".json" else yaml.safe_load(text)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", type=Path, help="SAX netlist (.json/.yml)")
    ap.add_argument("-o", "--output", type=Path, help="Output YAML (default: stdout)")
    ap.add_argument("--mode", choices=["routes", "connections", "nets"], default="routes")
    ap.add_argument("--dx", type=float, default=200.0, help="Placement spacing (um)")
    ap.add_argument("--port-map", type=json.loads, default=None,
                    help='JSON dict, e.g. \'{"in":"o1","out":"o2"}\'')
    ap.add_argument("--no-auto-ports", action="store_true",
                    help="Disable in0/out1 -> oN renaming")
    ap.add_argument("--cross-section", default=None)
    ap.add_argument("--name", default=None)
    args = ap.parse_args()

    result = sax_to_gdsfactory(
        _load(args.input), mode=args.mode, port_map=args.port_map,
        auto_port_names=not args.no_auto_ports, dx=args.dx,
        cross_section=args.cross_section, name=args.name,
    )
    text = yaml.safe_dump(result, sort_keys=False, default_flow_style=False)
    if args.output:
        args.output.write_text(text)
    else:
        print(text)


if __name__ == "__main__":
    main()
