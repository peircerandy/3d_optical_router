#!/usr/bin/env python3
"""Convert a SAX netlist into a gdsfactory YAML netlist.

SAX netlist (dict, or JSON/YAML file):
    {
      "instances": {"lft": "mmi1x2",                       # str form
                    "osu_0_0": "OSU",                      # custom component
                    "top": {"component": "straight",      # dict form
                            "settings": {"length": 10}}},
      "connections": {"lft,out0": "top,in0", ...},
      "ports": {"in0": "lft,in", "out0": "rgt,out"},
    }

Custom components
-----------------
SAX netlists may use components that do not exist in the gdsfactory PDK
(e.g. ``OSU`` = optical switching unit).  ``COMPONENT_LIBRARY`` maps those
names to real gdsfactory cells and says how their ports are numbered.  The
default maps ``OSU`` to ``mzi2x2_2x2`` (2x2 MZI switch: o1,o2 in / o3,o4 out).
Override or add entries from the CLI:

    --component-map '{"OSU": "mzi2x2_2x2"}'
    --component-map '{"OSU": {"component": "my_osu", "n_in": 2,
                              "settings": {"delta_length": 10}}}'

Usage:
    python sax2gf_yaml.py crossbar8x8.json -o crossbar8x8.pic.yml
    python sax2gf_yaml.py netlist.yml --mode nets --dx 150
"""
from __future__ import annotations

import argparse
import importlib
import importlib.util
import json
import math
import re
import sys
from pathlib import Path
from typing import Any, Callable

import yaml

# Default SAX -> gdsfactory port-name map for ordinary (non-library) components.
DEFAULT_PORT_MAP = {
    "in": "o1", "out": "o2",
    "in0": "o1", "in1": "o2", "out0": "o3", "out1": "o4",
}

# SAX component name (case-insensitive) -> gdsfactory component + port layout.
#   component : name of a cell in the active gdsfactory PDK
#   n_in      : number of input ports; outputs are numbered after the inputs
#               (in0,in1 -> o1,o2 ; out0,out1 -> o3,o4)
#   settings  : default settings (instance settings override these)
COMPONENT_LIBRARY: dict[str, dict[str, Any]] = {
    "osu": {"component": "mzi2x2_2x2", "n_in": 2, "settings": {}},
    "optical_switching_unit": {"component": "mzi2x2_2x2", "n_in": 2, "settings": {}},
}

# SAX-style port names: in0/out1, In1/Out2, I1/I2, O1/O2 (case-insensitive).
# Lowercase o<N> is treated as an already-correct gdsfactory name instead.
_IO_RE = re.compile(r"(in|out|i|o)(\d*)", re.IGNORECASE)


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def _split(ref: str) -> tuple[str, str]:
    """'inst,port' -> ('inst', 'port'). SAX uses a comma separator."""
    inst, _, port = str(ref).partition(",")
    if not port:
        raise ValueError(f"Bad port reference {ref!r}; expected 'instance,port'")
    return inst.strip(), port.strip()


def _generic_port(port: str, port_map: dict[str, str], auto: bool) -> str:
    if port in port_map:
        return port_map[port]
    if re.fullmatch(r"[oO]\d+", port):  # gdsfactory-style names are lowercase
        return port.lower()
    if auto:
        m = re.fullmatch(r"(?:in|out|p|port)?(\d+)", port)
        if m:
            return f"o{int(m.group(1)) + 1}"
    return port


def _normalize_library(user_map: dict[str, Any] | None) -> dict[str, dict[str, Any]]:
    lib = {k.lower(): dict(v) for k, v in COMPONENT_LIBRARY.items()}
    for key, val in (user_map or {}).items():
        if isinstance(val, str):
            val = {"component": val}
        entry = {**lib.get(key.lower(), {"n_in": 2, "settings": {}}), **val}
        lib[key.lower()] = entry
    return lib


def _sax_component(spec: Any) -> str:
    if isinstance(spec, str):
        return spec
    if isinstance(spec, dict):
        comp = spec.get("component") or spec.get("model")
        if comp is not None:
            return str(comp)
    raise ValueError(f"Instance has no 'component': {spec!r}")


def _convert_instance(spec: Any, lib: dict[str, dict[str, Any]]) -> dict[str, Any]:
    sax_name = _sax_component(spec)
    entry = lib.get(sax_name.lower())
    out: dict[str, Any] = {"component": entry["component"] if entry else sax_name}
    settings: dict[str, Any] = dict(entry.get("settings", {})) if entry else {}
    if isinstance(spec, dict):
        settings.update(spec.get("settings") or {})
    if settings:
        out["settings"] = settings
    return out


def _index_base(refs: list[str]) -> int:
    """0 if any in/out port is numbered 0 among these refs, else 1."""
    nums = []
    for r in refs:
        p = _split(r)[1]
        if re.fullmatch(r"o\d+", p):  # gdsfactory name, not SAX numbering
            continue
        m = _IO_RE.fullmatch(p)
        if m and m.group(2) != "":
            nums.append(int(m.group(2)))
    return 0 if (not nums or min(nums) == 0) else 1


def _library_port(port: str, n_in: int, base: int) -> str:
    """Inputs (in/i) -> o(idx+1); outputs (out/o) -> o(n_in+idx+1).

    e.g. I1,I2,O1,O2 (or in0,in1,out0,out1) -> o1,o2,o3,o4 for a 2x2.
    Lowercase o<N> is assumed to already be a gdsfactory port name.
    """
    if re.fullmatch(r"o\d+", port):
        return port
    m = _IO_RE.fullmatch(port)
    if not m:
        return port
    kind, num = m.group(1).lower(), m.group(2)
    idx = (int(num) - base) if num != "" else 0
    return f"o{idx + 1}" if kind in ("in", "i") else f"o{n_in + idx + 1}"


def _all_refs(netlist: dict[str, Any], conns: dict[str, str]) -> list[str]:
    refs = list(conns) + list(conns.values())
    refs += [r for r in (netlist.get("ports") or {}).values()]
    return [str(r) for r in refs]


# --------------------------------------------------------------------------- #
# netlist discovery
# --------------------------------------------------------------------------- #
_PLACEMENT_KEYS = ("x", "y", "dx", "dy", "rotation", "mirror", "xmin", "ymin",
                   "xmax", "ymax", "port")


def _find_netlist(data: Any, circuit: str | None = None) -> dict[str, Any]:
    """Locate the netlist dict that actually contains 'instances'."""
    if not isinstance(data, dict):
        raise ValueError(f"Netlist must be a mapping, got {type(data).__name__}")
    if circuit:
        if circuit not in data:
            raise KeyError(f"Circuit {circuit!r} not found; keys: {list(data)}")
        return _find_netlist(data[circuit])
    if isinstance(data.get("instances"), dict) and data["instances"]:
        return data
    candidates = {k: v for k, v in data.items()
                  if isinstance(v, dict) and isinstance(v.get("instances"), dict)}
    if not candidates:
        raise ValueError(
            "No 'instances' found in the netlist. Top-level keys: "
            f"{list(data)}. Pass the right circuit with --circuit if it is nested."
        )
    used = {
        (s.get("component") if isinstance(s, dict) else s)
        for v in candidates.values() for s in v["instances"].values()
    }
    tops = [k for k in candidates if k not in used]
    chosen = tops[-1] if tops else list(candidates)[-1]
    if len(candidates) > 1:
        print(f"[sax2gf] multiple circuits {list(candidates)}; using {chosen!r}")
    return candidates[chosen]


def _normalize_connections(conns: Any) -> dict[str, str]:
    """Accept dict {'a,p': 'b,q'} or a list of [a, b] pairs."""
    if not conns:
        return {}
    if isinstance(conns, dict):
        return {str(a): str(b) for a, b in conns.items()}
    return {str(a): str(b) for a, b in conns}


# --------------------------------------------------------------------------- #
# placement topologies
# --------------------------------------------------------------------------- #
# A placer is  f(names, edges, px, py, opts) -> {instance: (x, y)}  where
#   names : instance names in netlist order
#   edges : [(src_instance, dst_instance), ...] taken from the SAX connections
#   px,py : pitch between neighbouring cells in um
#   opts  : {"pattern": regex-or-None}  (how to read indices from names)
# Add your own with @register_topology("name") or --topology file.py:function.
Placer = Callable[[list, list, float, float, dict], dict]
TOPOLOGIES: dict[str, Placer] = {}


class TopologyError(ValueError):
    """Raised when a topology cannot be applied to this netlist."""


def register_topology(*names: str):
    def deco(fn: Placer) -> Placer:
        for n in names:
            TOPOLOGIES[n] = fn
        return fn
    return deco


def _parse_indices(name: str, pattern: str | None, keys: tuple[str, str]):
    """Read two integers out of an instance name, or None.

    With a --name-pattern regex the named groups `keys` are used, e.g.
    '(?P<row>\\d+)_(?P<col>\\d+)$'.  Without one: the last two integers
    (osu_3_5 -> 3,5) or one even-length integer split in half (OSU35 -> 3,5).
    """
    if pattern:
        m = re.search(pattern, name)
        if m and all(m.groupdict().get(k) is not None for k in keys):
            return tuple(int(m.group(k)) for k in keys)
        return None
    toks = re.findall(r"\d+", name)
    if len(toks) >= 2:
        return int(toks[-2]), int(toks[-1])
    if len(toks) == 1 and len(toks[0]) >= 2 and len(toks[0]) % 2 == 0:
        h = len(toks[0]) // 2
        return int(toks[0][:h]), int(toks[0][h:])
    return None


def _place_unmatched(pos, names, edges, px, py):
    """Put instances without indices (couplers, sources, ...) beside the array.

    A cell fed by a placed cell (sink) goes below the array if that cell is on
    the bottom row, else to the right.  One feeding a placed cell (source)
    goes left of the array if that cell is in the first column, else above.
    Each sits in line with its neighbour.
    """
    pending = [n for n in names if n not in pos]
    if not pending:
        return
    xs = [p[0] for p in pos.values()] or [0.0]
    ys = [p[1] for p in pos.values()] or [0.0]
    xmin, xmax, ymin, ymax = min(xs), max(xs), min(ys), max(ys)
    taken = set(pos.values())

    def put(n, x, y, step):
        while (x, y) in taken:
            x, y = x + step[0], y + step[1]
        pos[n] = (x, y)
        taken.add((x, y))
        pending.remove(n)

    while pending:
        progress = False
        for n in list(pending):
            fed_by = [s for s, d in edges if d == n and s in pos]
            feeds = [d for s, d in edges if s == n and d in pos]
            if fed_by:  # sink
                x, y = pos[fed_by[0]]
                if y == ymin:
                    put(n, x, ymin - py, (0.0, -py / 2))
                else:
                    put(n, xmax + px, y, (0.0, -py / 2))
            elif feeds:  # source
                x, y = pos[feeds[0]]
                if x == xmin:
                    put(n, xmin - px, y, (0.0, -py / 2))
                else:
                    put(n, x, ymax + py, (px / 2, 0.0))
            else:
                continue
            progress = True
        if not progress:  # isolated / chained to other unplaced nodes
            for k, n in enumerate(list(pending), 1):
                put(n, xmax + px, ymin - k * py, (0.0, -py / 2))
            return


def _indexed_layout(names, edges, px, py, opts, keys, center_columns):
    idx = {n: _parse_indices(n, opts.get("pattern"), keys) for n in names}
    matched = {n: v for n, v in idx.items() if v is not None}
    if not matched:
        raise TopologyError(
            f"no instance name yields {keys}; pass --name-pattern with named "
            f"groups {keys}, e.g. '(?P<{keys[0]}>\\d+)_(?P<{keys[1]}>\\d+)$'")
    if len(set(matched.values())) != len(matched):
        raise TopologyError(f"two instances map to the same {keys}; "
                            "pass a more specific --name-pattern")
    a_min = min(v[0] for v in matched.values())
    b_min = min(v[1] for v in matched.values())
    pos: dict[str, tuple[float, float]] = {}
    if center_columns:  # stages: column = stage, row = index, columns centred
        cols: dict[int, list] = {}
        for n, (s, i) in matched.items():
            cols.setdefault(s, []).append((i, n))
        tallest = max(len(v) for v in cols.values())
        for s, items in cols.items():
            items.sort()
            off = (tallest - len(items)) / 2
            for k, (_, n) in enumerate(items):
                pos[n] = ((s - a_min) * px, -(k + off) * py)
    else:  # grid: first index = row (down), second = column (right)
        for n, (r, c) in matched.items():
            pos[n] = ((c - b_min) * px, -(r - a_min) * py)
    _place_unmatched(pos, names, edges, px, py)
    return pos


@register_topology("grid", "crossbar")
def _grid(names, edges, px, py, opts):
    """Rectangular array: (row, col) from the instance name. Crossbars, meshes."""
    return _indexed_layout(names, edges, px, py, opts, ("row", "col"), False)


@register_topology("stages", "benes", "banyan", "butterfly", "clos")
def _stages(names, edges, px, py, opts):
    """Multistage network: column per stage, row per index within the stage."""
    return _indexed_layout(names, edges, px, py, opts, ("stage", "index"), True)


@register_topology("layered")
def _layered(names, edges, px, py, opts):
    """Generic left-to-right layout from connectivity; reduces wire crossings.

    Columns = longest path from the sources; within a column instances are
    ordered by repeated barycentre sweeps (neighbours' average height).
    """
    pos_in_names = {n: i for i, n in enumerate(names)}
    succ = {n: [] for n in names}
    pred = {n: [] for n in names}
    for a, b in edges:
        if a in succ and b in succ and a != b and b not in succ[a]:
            succ[a].append(b)
            pred[b].append(a)
    indeg = {n: len(pred[n]) for n in names}
    layer: dict[str, int] = {}
    remaining = set(names)
    ready = [n for n in names if indeg[n] == 0]
    while remaining:
        if not ready:  # cycle: break it at the least-constrained node
            ready = [min(remaining, key=lambda k: (indeg[k], pos_in_names[k]))]
        n = ready.pop(0)
        if n not in remaining:
            continue
        remaining.discard(n)
        layer.setdefault(n, 0)
        for m in succ[n]:
            if m in remaining:
                layer[m] = max(layer.get(m, 0), layer[n] + 1)
                indeg[m] -= 1
                if indeg[m] == 0:
                    ready.append(m)

    layers: dict[int, list] = {}
    for n in names:
        layers.setdefault(layer[n], []).append(n)
    yc: dict[str, float] = {}

    def recentre(k):
        L = layers[k]
        for i, n in enumerate(L):
            yc[n] = i - (len(L) - 1) / 2

    for k in layers:
        recentre(k)
    keys = sorted(layers)
    for sweep in range(6):
        forward = sweep % 2 == 0
        nbrs = pred if forward else succ
        for k in (keys if forward else keys[::-1]):
            def bary(n):
                v = [yc[m] for m in nbrs[n] if layer[m] != k]
                return sum(v) / len(v) if v else yc[n]
            layers[k].sort(key=bary)
            recentre(k)
    return {n: (layer[n] * px, -yc[n] * py) for n in names}


@register_topology("auto")
def _auto(names, edges, px, py, opts):
    """Grid if every instance is a unique (row, col) filling a rectangle,
    otherwise the generic layered layout."""
    idx = [_parse_indices(n, opts.get("pattern"), ("row", "col")) for n in names]
    if (len(names) >= 4 and all(v is not None for v in idx)
            and len(set(idx)) == len(idx)
            and len({v[0] for v in idx}) * len({v[1] for v in idx}) == len(idx)):
        print("[sax2gf] auto topology: grid")
        return _grid(names, edges, px, py, opts)
    print("[sax2gf] auto topology: layered")
    return _layered(names, edges, px, py, opts)


def _load_topology(spec: str | Placer) -> Placer:
    """Name from TOPOLOGIES, a callable, or 'module_or_file.py:function'."""
    if callable(spec):
        return spec
    if spec in TOPOLOGIES:
        return TOPOLOGIES[spec]
    if ":" in spec:
        target, _, func = spec.rpartition(":")
        if target.endswith(".py"):
            s = importlib.util.spec_from_file_location("sax2gf_user_topology", target)
            mod = importlib.util.module_from_spec(s)
            s.loader.exec_module(mod)
        else:
            sys.path.insert(0, ".")
            mod = importlib.import_module(target)
        return getattr(mod, func)
    raise ValueError(f"Unknown topology {spec!r}. Built-in: {sorted(TOPOLOGIES)}; "
                     "or use 'file.py:function'.")


def _component_size(comp: str, settings: dict | None, _cache: dict = {}):
    """(width, height) in um from gdsfactory, or None if unavailable."""
    key = (comp, json.dumps(settings or {}, sort_keys=True, default=str))
    if key not in _cache:
        try:
            import gdsfactory as gf
            c = gf.get_component(comp, **(settings or {}))
            w = getattr(c, "dxsize", None) or getattr(c, "xsize", None)
            h = getattr(c, "dysize", None) or getattr(c, "ysize", None)
            _cache[key] = (float(w), float(h))
        except Exception:
            _cache[key] = None
    return _cache[key]


def _resolve_pitch(instances, dx, dy, auto_pitch, margin):
    px, py = dx, dy
    if auto_pitch and (px is None or py is None):
        sizes = [s for s in (_component_size(v["component"], v.get("settings"))
                             for v in instances.values()) if s]
        if sizes:
            px = px if px is not None else max(s[0] for s in sizes) + margin
            py = py if py is not None else max(s[1] for s in sizes) + margin
            print(f"[sax2gf] auto pitch: {px:.0f} x {py:.0f} um")
        else:
            print("[sax2gf] --auto-pitch needs gdsfactory and known components; "
                  "using defaults")
    px = 200.0 if px is None else px
    py = px if py is None else py
    return px, py


def _warn_unknown_components(instances: dict[str, dict[str, Any]]) -> None:
    """If gdsfactory is importable, warn about components missing from the PDK."""
    try:
        import gdsfactory as gf
        cells = set(gf.get_active_pdk().cells)
    except Exception:  # gdsfactory not installed / no PDK: skip silently
        return
    missing = sorted({v["component"] for v in instances.values()} - cells)
    if missing:
        print(f"[sax2gf] WARNING: not in active PDK {gf.get_active_pdk().name!r}: "
              f"{missing}. Add them with --component-map.")


# --------------------------------------------------------------------------- #
# routing
# --------------------------------------------------------------------------- #
ROUTE_GROUPS = ("none", "port", "link")


def _build_routes(
    links: dict[str, str],
    strategy: str,
    settings: dict[str, Any],
    group: str = "none",
    rules: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    """Build the gdsfactory `routes:` section.

    Every gdsfactory route group has its own routing_strategy and settings.
      group='none' -> all links in one bundle (default)
      group='port' -> one bundle per source port (e.g. all o3 links, all o4 links)
      group='link' -> every link routed on its own (most robust for dense
                      meshes where a single bundle fails to sort/route)
    rules: [{"name": "vertical", "match": "o4->.*o2$", "strategy": "route_bundle",
             "settings": {"radius": 10}}, ...]; `match` is a regex searched in
             "src,port->dst,port" (gdsfactory port names); first match wins and
             its strategy/settings override the defaults.
    """
    if group not in ROUTE_GROUPS:
        raise ValueError(f"route group must be one of {ROUTE_GROUPS}, got {group!r}")
    buckets: dict[str, dict[str, Any]] = {}

    def bucket(key, strat, sett):
        if key not in buckets:
            buckets[key] = {"strategy": strat, "settings": sett, "links": {}}
        return buckets[key]

    for i, (a, b) in enumerate(links.items()):
        text = f"{a}->{b}"
        rule = next((r for r in (rules or []) if re.search(r["match"], text)), None)
        if rule is not None:
            key = f"rule_{rule.get('name', rules.index(rule))}"
            bkt = bucket(key, rule.get("strategy", strategy),
                         {**settings, **(rule.get("settings") or {})})
        else:
            key = {"none": "optical",
                   "port": f"optical_{_split(a)[1]}",
                   "link": f"link_{i:04d}"}[group]
            bkt = bucket(key, strategy, dict(settings))
        bkt["links"][a] = b

    routes = {}
    for key, bkt in buckets.items():
        r: dict[str, Any] = {"links": bkt["links"], "routing_strategy": bkt["strategy"]}
        if bkt["settings"]:
            r["settings"] = bkt["settings"]
        routes[key] = r
    return routes


# --------------------------------------------------------------------------- #
# main conversion
# --------------------------------------------------------------------------- #
def sax_to_gdsfactory(
    netlist: dict[str, Any],
    mode: str = "routes",
    port_map: dict[str, str] | None = None,
    auto_port_names: bool = True,
    dx: float | None = None,
    dy: float | None = None,
    topology: str | Placer = "auto",
    name_pattern: str | None = None,
    layout: dict[str, Any] | None = None,
    auto_pitch: bool = False,
    margin: float = 50.0,
    routing_strategy: str = "route_bundle",
    cross_section: str | None = None,
    name: str | None = None,
    circuit: str | None = None,
    component_map: dict[str, Any] | None = None,
    rename_top_ports: bool = False,
    route_settings: dict[str, Any] | None = None,
    route_group: str = "none",
    route_rules: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    """Return a gdsfactory-compatible netlist dict.

    mode:
      'routes'      -> SAX connections become routing links (default).
      'connections' -> abutting gdsfactory 'connections' (no routing).
      'nets'        -> schematic-style 'nets' list.

    topology: how to place instances (see TOPOLOGIES): 'auto', 'grid'/'crossbar',
      'stages'/'benes'/..., 'layered', a callable, or 'file.py:function'.
    dx, dy: cell pitch in um (default 200; or component size + margin with
      auto_pitch=True when gdsfactory is installed).
    layout: {instance: {"x":..,"y":..,"rotation":..}} overriding any placement.

    Routing (mode='routes'):
      routing_strategy: gdsfactory routing function used by from_yaml, e.g.
        'route_bundle' (default), 'route_bundle_all_angle', 'route_bundle_sbend',
        'route_bundle_path_length_match' (names depend on your gdsfactory version).
      route_settings: extra router settings, e.g. {"radius": 10, "separation": 6}.
      route_group: 'none' | 'port' | 'link' (see _build_routes).
      route_rules: per-link overrides of strategy/settings (see _build_routes).
    """
    pm = {**DEFAULT_PORT_MAP, **(port_map or {})}
    lib = _normalize_library(component_map)

    netlist = _find_netlist(netlist, circuit)
    raw_conns = _normalize_connections(netlist.get("connections", {}))
    sax_comp = {i: _sax_component(s) for i, s in netlist["instances"].items()}
    instances = {i: _convert_instance(s, lib) for i, s in netlist["instances"].items()}
    _warn_unknown_components(instances)

    # Port-numbering base (0- or 1-indexed) per library component type.
    refs = _all_refs(netlist, raw_conns)
    base_by_type: dict[str, int] = {}
    for t in {c.lower() for c in sax_comp.values() if c.lower() in lib}:
        base_by_type[t] = _index_base(
            [r for r in refs if sax_comp.get(_split(r)[0], "").lower() == t])

    def conv(ref: str) -> str:
        inst, port = _split(ref)
        t = sax_comp.get(inst, "").lower()
        if t in lib:
            return f"{inst},{_library_port(port, lib[t].get('n_in', 2), base_by_type[t])}"
        return f"{inst},{_generic_port(port, pm, auto_port_names)}"

    # Placements: keep any given in the input; auto-place the rest.
    given = {
        k: {pk: pv for pk, pv in v.items() if pk in _PLACEMENT_KEYS}
        for k, v in (netlist.get("placements") or {}).items()
        if isinstance(v, dict)
    }
    for iname, spec in netlist["instances"].items():
        if isinstance(spec, dict):
            inline = {k: spec[k] for k in _PLACEMENT_KEYS if k in spec}
            if inline:
                given.setdefault(iname, {}).update(inline)
    for k, v in (layout or {}).items():  # explicit user layout wins over all
        if isinstance(v, dict):
            given.setdefault(k, {}).update(
                {pk: pv for pk, pv in v.items() if pk in _PLACEMENT_KEYS})

    px, py = _resolve_pitch(instances, dx, dy, auto_pitch, margin)
    edges = [(_split(a)[0], _split(b)[0]) for a, b in raw_conns.items()]
    placer = _load_topology(topology)
    opts = {"pattern": name_pattern}
    try:
        pos = placer(list(instances), edges, px, py, opts)
    except TopologyError as e:
        print(f"[sax2gf] WARNING: topology {topology!r} not applicable ({e}); "
              "using 'layered'")
        pos = _layered(list(instances), edges, px, py, opts)
    auto = {n: {"x": float(pos[n][0]) + 0.0, "y": float(pos[n][1]) + 0.0}  # +0.0: no -0.0
            for n in instances}
    placements = {n: {**auto[n], **given.get(n, {})} for n in instances}

    out: dict[str, Any] = {}
    if name:
        out["name"] = name
    out["instances"] = instances
    out["placements"] = placements

    connections = {conv(a): conv(b) for a, b in raw_conns.items()}
    if mode == "routes":
        settings = dict(route_settings or {})
        if cross_section:
            settings.setdefault("cross_section", cross_section)
        out["routes"] = _build_routes(connections, routing_strategy, settings,
                                      route_group, route_rules)
    elif mode == "connections":
        out["connections"] = connections
    elif mode == "nets":
        out["nets"] = [[a, b] for a, b in connections.items()]
    else:
        raise ValueError(f"Unknown mode {mode!r}")

    # Circuit-level ports. Original names are kept by default: renaming
    # in0..in7/out0..out7 to oN collides for larger circuits like a crossbar.
    top_ports = netlist.get("ports") or {}
    if rename_top_ports:
        ports = {f"o{k}": conv(ref) for k, (_, ref) in enumerate(top_ports.items(), 1)}
    else:
        ports = {str(p): conv(ref) for p, ref in top_ports.items()}
    if ports:
        out["ports"] = ports
    return out


# --------------------------------------------------------------------------- #
# GDS export with labelled system ports
# --------------------------------------------------------------------------- #
# gdsfactory's YAML has no way to put text on a port, so labelling happens right
# after gf.read.from_yaml().  The circuit-level `ports:` of the YAML are the
# system inputs/outputs; each one gets a GDS text label at its location.
_IN_RE = re.compile(r"(?i)^(in|input|i)[\W_]*\d*$")
_OUT_RE = re.compile(r"(?i)^(out|output|o)[\W_]*\d*$")


def port_direction(name: str) -> str:
    """'IN' / 'OUT' guessed from a system-port name (in0, I1, out3, O2), else ''."""
    base = re.sub(r"^.*[,:/]", "", str(name))
    if re.fullmatch(r"o\d+", base):  # lowercase oN is gdsfactory's neutral naming
        return ""
    if _IN_RE.match(base):
        return "IN"
    if _OUT_RE.match(base):
        return "OUT"
    return ""


def _iter_ports(ports):
    try:
        return list(ports.values())  # gdsfactory <= 8 (dict-like)
    except AttributeError:
        return list(ports)  # gdsfactory 9 (Ports object)


def _port_xy(p) -> tuple[float, float]:
    for attr in ("dcenter", "center"):  # dcenter = um in gdsfactory 9
        v = getattr(p, attr, None)
        if v is not None:
            return float(v[0]), float(v[1])
    raise AttributeError(f"cannot read the position of port {p!r}")


def label_ports(
    component,
    layer: Any = "TEXT",
    text_format: str = "{name}",
    text_polygons: bool = False,
    text_size: float = 10.0,
):
    """Return a component whose ports carry a GDS text label at their position.

    text_format may use {name} and {direction} (IN/OUT from the port name),
    e.g. "{direction}:{name}" -> "IN:in0".  With text_polygons=True the text is
    also drawn as geometry next to the port (on `layer`), which is visible even
    in viewers that hide label records.
    The input component is not modified (it may be locked/cached); a wrapper
    component with the same ports is returned.
    """
    import gdsfactory as gf

    top = gf.Component()
    ref = top << component
    top.add_ports(ref.ports)
    for p in _iter_ports(ref.ports):
        x, y = _port_xy(p)
        name = str(p.name)
        text = text_format.format(name=name, direction=port_direction(name))
        for lyr in (layer, (66, 0)):  # fall back to the generic TEXT layer
            try:
                try:
                    top.add_label(text=text, position=(x, y), layer=lyr)
                except TypeError:
                    top.add_label(text, (x, y), layer=lyr)
                break
            except (KeyError, ValueError):
                continue
        if text_polygons:
            try:
                t = top << gf.components.text(text=text, size=text_size, layer=layer)
                ori = math.radians(float(getattr(p, "orientation", 0) or 0))
                move = getattr(t, "dmove", None) or t.move
                move((x + 1.5 * text_size * math.cos(ori),
                      y + 1.5 * text_size * math.sin(ori)))
            except Exception as e:  # cosmetic only; never fail the export
                print(f"[sax2gf] could not draw text polygons for {name}: {e}")
    return top


def yaml_to_gds(yaml_path, gds_path=None, label: bool = True, **label_kwargs):
    """gf.read.from_yaml -> label the system ports -> write GDS. Returns the component.

    Drop-in for ``gf.read.from_yaml(path)`` in your own script:
        from sax2gf_yaml import yaml_to_gds
        c = yaml_to_gds("crossbar8x8.pic.yml", "crossbar8x8.gds")
    """
    import gdsfactory as gf

    c = gf.read.from_yaml(str(yaml_path))
    if label:
        c = label_ports(c, **label_kwargs)
    if gds_path:
        c.write_gds(str(gds_path))
        print(f"[sax2gf] wrote {gds_path} ({len(_iter_ports(c.ports))} labelled ports)")
    return c


def _load(path: Path) -> dict[str, Any]:
    text = path.read_text()
    return json.loads(text) if path.suffix == ".json" else yaml.safe_load(text)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", type=Path, help="SAX netlist (.json/.yml)")
    ap.add_argument("-o", "--output", type=Path, help="Output YAML (default: stdout)")
    ap.add_argument("--mode", choices=["routes", "connections", "nets"], default="routes")
    ap.add_argument("--topology", default="auto",
                    help="Placement topology: " + ", ".join(sorted(TOPOLOGIES)) +
                         ", or 'file.py:function' for your own (default: auto)")
    ap.add_argument("--name-pattern", default=None,
                    help="Regex with named groups to read indices from instance "
                         "names: (?P<row>..)(?P<col>..) for grid/crossbar, "
                         "(?P<stage>..)(?P<index>..) for stages")
    ap.add_argument("--layout", type=Path, default=None,
                    help="JSON/YAML {instance: {x, y, rotation}} overriding placement")
    ap.add_argument("--dx", type=float, default=None,
                    help="Horizontal pitch between cells in um (default 200)")
    ap.add_argument("--dy", type=float, default=None,
                    help="Vertical pitch in um (default = dx)")
    ap.add_argument("--auto-pitch", action="store_true",
                    help="Pitch = largest cell size + --margin (needs gdsfactory)")
    ap.add_argument("--margin", type=float, default=50.0,
                    help="Routing gap added by --auto-pitch (um)")
    ap.add_argument("--port-map", type=json.loads, default=None,
                    help='JSON dict for ordinary components, e.g. \'{"in":"o1"}\'')
    ap.add_argument("--component-map", type=json.loads, default=None,
                    help='JSON dict of custom SAX components -> gdsfactory cells, '
                         'e.g. \'{"OSU": "mzi2x2_2x2"}\'')
    ap.add_argument("--no-auto-ports", action="store_true",
                    help="Disable in0/out1 -> oN renaming for ordinary components")
    ap.add_argument("--rename-top-ports", action="store_true",
                    help="Rename circuit-level ports to o1, o2, ... (default: keep names)")
    ap.add_argument("--routing-strategy", default="route_bundle",
                    help="gdsfactory routing strategy for all links, e.g. route_bundle "
                         "(default), route_bundle_all_angle, route_bundle_sbend, "
                         "route_bundle_path_length_match (depends on gdsfactory version)")
    ap.add_argument("--route-settings", type=json.loads, default=None,
                    help='JSON settings passed to the router, e.g. '
                         '\'{"radius": 10, "separation": 6}\'')
    ap.add_argument("--route-group", choices=ROUTE_GROUPS, default="none",
                    help="none: one bundle; port: one bundle per source port; "
                         "link: route each link separately")
    ap.add_argument("--route-rules", type=Path, default=None,
                    help="JSON/YAML list of {name, match, strategy, settings}; "
                         "'match' is a regex on 'src,port->dst,port'; first match wins")
    ap.add_argument("--gds", type=Path, default=None,
                    help="Also build this GDS with gdsfactory (labels the system "
                         "ports); needs gdsfactory installed")
    ap.add_argument("--no-port-labels", action="store_true",
                    help="Do not add text labels to the system ports in the GDS")
    ap.add_argument("--label-layer", default="TEXT",
                    help="Layer for port labels: name (TEXT) or 'layer,datatype' "
                         "(default TEXT)")
    ap.add_argument("--label-format", default="{name}",
                    help="Label text; {name} and {direction} (IN/OUT) available, "
                         "e.g. '{direction}:{name}'")
    ap.add_argument("--label-polygons", action="store_true",
                    help="Also draw the label as visible text geometry near the port")
    ap.add_argument("--label-size", type=float, default=10.0,
                    help="Text height in um for --label-polygons")
    ap.add_argument("--cross-section", default=None)
    ap.add_argument("--name", default=None)
    ap.add_argument("--circuit", default=None,
                    help="Key of the circuit to convert if the netlist is nested")
    args = ap.parse_args()

    result = sax_to_gdsfactory(
        _load(args.input), mode=args.mode, port_map=args.port_map,
        auto_port_names=not args.no_auto_ports, dx=args.dx, dy=args.dy,
        topology=args.topology, name_pattern=args.name_pattern,
        layout=_load(args.layout) if args.layout else None,
        auto_pitch=args.auto_pitch, margin=args.margin,
        routing_strategy=args.routing_strategy, route_settings=args.route_settings,
        route_group=args.route_group,
        route_rules=_load(args.route_rules) if args.route_rules else None,
        cross_section=args.cross_section, name=args.name, circuit=args.circuit,
        component_map=args.component_map, rename_top_ports=args.rename_top_ports,
    )
    text = yaml.safe_dump(result, sort_keys=False, default_flow_style=False)

    sys_ports = list(result.get("ports", {}))
    dirs = [port_direction(p) for p in sys_ports]
    if sys_ports:
        print(f"[sax2gf] system ports: {dirs.count('IN')} in, {dirs.count('OUT')} out, "
              f"{dirs.count('')} unclassified -> {sys_ports}")

    out_yaml = args.output or (args.input.with_suffix(".pic.yml") if args.gds else None)
    if out_yaml:
        out_yaml.write_text(text)
    else:
        print(text)

    if args.gds:
        layer = args.label_layer
        if "," in layer:
            layer = tuple(int(v) for v in layer.split(","))
        yaml_to_gds(out_yaml, args.gds, label=not args.no_port_labels, layer=layer,
                    text_format=args.label_format, text_polygons=args.label_polygons,
                    text_size=args.label_size)


if __name__ == "__main__":
    main()
