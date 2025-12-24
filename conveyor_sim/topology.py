from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Set, Tuple, Optional, Iterable
from collections import deque

from .grid import Grid, TRACK_TOKENS
from .models import Coord, Diverter, DiverterSide

DIRS = {
    "U": (-1, 0),
    "D": (1, 0),
    "L": (0, -1),
    "R": (0, 1),
}
OPP = {"U": "D", "D": "U", "L": "R", "R": "L"}


def _add(rc: Coord, d: str) -> Coord:
    dr, dc = DIRS[d]
    return (rc[0] + dr, rc[1] + dc)


def _is_track_token(tok: str) -> bool:
    return tok in TRACK_TOKENS


def _pipe_is_vertical_straight(grid: Grid, rc: Coord) -> bool:
    """Return True if a '|' cell should be treated as vertical-straight (no turn)."""
    if grid.get(rc) != "|":
        return False
    up = _add(rc, "U")
    dn = _add(rc, "D")
    # Potential vertical connection exists if neighbor is a track token that is not '='
    pot_up = grid.in_bounds(up) and _is_track_token(grid.get(up)) and grid.get(up) != "="
    pot_dn = grid.in_bounds(dn) and _is_track_token(grid.get(dn)) and grid.get(dn) != "="
    return pot_up and pot_dn


def _allowed_dirs(grid: Grid, rc: Coord) -> Set[str]:
    tok = grid.get(rc)
    if tok == "=":
        return {"L", "R"}
    if tok == "|":
        if _pipe_is_vertical_straight(grid, rc):
            return {"U", "D"}
        # corner-ish: allow existing vertical + horizontal to non-straight '|'
        allowed: Set[str] = set()
        for d in ("U", "D"):
            nb = _add(rc, d)
            if grid.in_bounds(nb) and _is_track_token(grid.get(nb)) and grid.get(nb) != "=":
                allowed.add(d)
        for d in ("L", "R"):
            nb = _add(rc, d)
            if not (grid.in_bounds(nb) and _is_track_token(grid.get(nb))):
                continue
            nb_tok = grid.get(nb)
            if nb_tok == "|":
                # Only connect horizontally to a corner-pipe, not a straight vertical pipe.
                if _pipe_is_vertical_straight(grid, nb):
                    continue
            allowed.add(d)
        return allowed

    if tok in {"C", "S", "I"}:
        # Flexible. Heuristic: prefer straight if obvious.
        neighbors: Dict[str, Coord] = {}
        for d in ("U", "D", "L", "R"):
            nb = _add(rc, d)
            if not (grid.in_bounds(nb) and _is_track_token(grid.get(nb))):
                continue
            # Never directly connect C-C (those are diverter pair adjacency, not belt adjacency).
            if tok == "C" and grid.get(nb) == "C":
                continue
            if grid.get(nb) == "C" and tok == "C":
                continue
            neighbors[d] = nb

        # Determine possible horizontal/vertical based on neighbor token types.
        has_h = any(d in neighbors for d in ("L", "R"))
        has_v = any(d in neighbors for d in ("U", "D"))

        if has_h and not has_v:
            return {"L", "R"} & set(neighbors.keys())
        if has_v and not has_h:
            return {"U", "D"} & set(neighbors.keys())

        # Both exist: choose best pair to keep degree=2.
        # If we have both L and R -> treat as horizontal
        if "L" in neighbors and "R" in neighbors:
            return {"L", "R"}
        # If we have both U and D -> treat as vertical
        if "U" in neighbors and "D" in neighbors:
            return {"U", "D"}

        # Otherwise, just use whatever two exist (corner-like)
        return set(neighbors.keys())
    return set()


def build_belt_graph(grid: Grid) -> Dict[Coord, Set[Coord]]:
    """Build an undirected graph for belt connectivity (excluding diverter cross-track adjacency)."""
    allowed: Dict[Coord, Set[str]] = {}
    for rc in grid.iter_track_coords():
        allowed[rc] = _allowed_dirs(grid, rc)

    graph: Dict[Coord, Set[Coord]] = {rc: set() for rc in allowed.keys()}

    for rc, dirs in allowed.items():
        for d in dirs:
            nb = _add(rc, d)
            if not (grid.in_bounds(nb) and nb in allowed):
                continue
            # Disallow direct C-C connection
            if grid.get(rc) == "C" and grid.get(nb) == "C":
                continue
            # mutual
            if OPP[d] in allowed[nb]:
                graph[rc].add(nb)
                graph[nb].add(rc)
    return graph


def _connected_components(graph: Dict[Coord, Set[Coord]]) -> List[Set[Coord]]:
    seen: Set[Coord] = set()
    comps: List[Set[Coord]] = []
    for node in graph:
        if node in seen:
            continue
        q = deque([node])
        seen.add(node)
        comp: Set[Coord] = {node}
        while q:
            cur = q.popleft()
            for nb in graph[cur]:
                if nb not in seen:
                    seen.add(nb)
                    comp.add(nb)
                    q.append(nb)
        comps.append(comp)
    return comps


def _order_cycle(comp: Set[Coord], graph: Dict[Coord, Set[Coord]]) -> List[Coord]:
    start = next(iter(comp))
    nbs = list(graph[start])
    if len(nbs) != 2:
        raise ValueError(f"Non-cycle node degree at {start}: {len(nbs)}")
    cycle = [start]
    prev = start
    cur = nbs[0]
    while True:
        cycle.append(cur)
        if cur == start:
            break
        nbs2 = list(graph[cur])
        if len(nbs2) != 2:
            raise ValueError(f"Non-cycle node degree at {cur}: {len(nbs2)}")
        nxt = nbs2[0] if nbs2[0] != prev else nbs2[1]
        prev, cur = cur, nxt
        if len(cycle) > len(comp) + 2:
            raise ValueError("Cycle ordering overflow; graph may not be a simple cycle.")
    # The last element repeats start; remove it
    cycle = cycle[:-1]
    if len(cycle) != len(comp):
        raise ValueError("Cycle ordering length mismatch.")
    return cycle


def _signed_area(coords: List[Coord]) -> float:
    # Use (x=col, y=-row) so y goes up.
    pts = [(c, -r) for r, c in coords]
    area = 0.0
    for i in range(len(pts)):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % len(pts)]
        area += x1 * y2 - x2 * y1
    return area / 2.0


@dataclass
class Topology:
    grid: Grid
    graph: Dict[Coord, Set[Coord]]
    loops: List[List[Coord]]
    outer_loop_id: int
    coord_to_loop_idx: Dict[Coord, Tuple[int, int]]  # coord -> (loop_id, idx)
    diverters: Dict[str, Diverter]

    @staticmethod
    def from_grid(grid: Grid, diverter_ticks_per_90: int = 1) -> "Topology":
        graph = build_belt_graph(grid)

        # Validate degrees: every track node must have degree 2 for pure cycles
        for rc, nbs in graph.items():
            if len(nbs) != 2:
                raise ValueError(
                    f"Track connectivity invalid at {rc} token='{grid.get(rc)}': degree={len(nbs)} (expected 2).\n"
                    "Hint: check ASCII layout around this cell."
                )

        comps = _connected_components(graph)
        if len(comps) != 2:
            raise ValueError(f"Expected 2 loop components (outer+inner), got {len(comps)}")

        loops: List[List[Coord]] = []
        for comp in comps:
            cyc = _order_cycle(comp, graph)
            # Ensure CCW orientation (positive area)
            if _signed_area(cyc) < 0:
                cyc = list(reversed(cyc))
            loops.append(cyc)

        # Identify outer loop by presence of 'I'
        i_coords = grid.coords_with("I")
        if len(i_coords) != 1:
            raise ValueError(f"Grid must contain exactly one 'I'. Found {len(i_coords)}")
        i_coord = i_coords[0]

        coord_to_loop_idx: Dict[Coord, Tuple[int, int]] = {}
        outer_loop_id = -1
        for lid, cyc in enumerate(loops):
            for idx, rc in enumerate(cyc):
                coord_to_loop_idx[rc] = (lid, idx)
            if i_coord in set(cyc):
                outer_loop_id = lid
        if outer_loop_id < 0:
            raise ValueError("Could not find outer loop containing I.")

        # Build diverters by pairing adjacent C cells
        c_coords = set(grid.coords_with("C"))
        used: Set[Coord] = set()
        diverters: Dict[str, Diverter] = {}
        did = 0
        for rc in sorted(c_coords):
            if rc in used:
                continue
            # find adjacent C
            adj = []
            for d in ("U", "D", "L", "R"):
                nb = _add(rc, d)
                if nb in c_coords:
                    adj.append(nb)
            if len(adj) != 1:
                raise ValueError(f"Each 'C' must pair with exactly one adjacent 'C'. At {rc} has {len(adj)} adjacent C.")
            other = adj[0]
            if other in used:
                raise ValueError(f"C pairing conflict at {rc} -> {other}")
            used.add(rc)
            used.add(other)

            # Determine which loop each belongs to
            if rc not in coord_to_loop_idx or other not in coord_to_loop_idx:
                raise ValueError("C cell not on any loop? Check layout.")
            lid1, idx1 = coord_to_loop_idx[rc]
            lid2, idx2 = coord_to_loop_idx[other]
            # NOTE: In some layouts, two adjacent C cells can belong to the same belt loop (e.g. folded tracks).
            # We allow this: diverters may connect two adjacent tracks even if they are part of the same loop component.

            # For stable labeling, side 'a' is outer loop if possible.
            if lid1 == outer_loop_id:
                a_coord, b_coord = rc, other
                a_lid, b_lid = lid1, lid2
                a_idx, b_idx = idx1, idx2
            elif lid2 == outer_loop_id:
                a_coord, b_coord = other, rc
                a_lid, b_lid = lid2, lid1
                a_idx, b_idx = idx2, idx1
            else:
                a_coord, b_coord = rc, other
                a_lid, b_lid = lid1, lid2
                a_idx, b_idx = idx1, idx2

            # Compute upstream/downstream per side
            def upstream_downstream(loop: List[Coord], idx: int) -> Tuple[Coord, Coord]:
                up = loop[(idx - 1) % len(loop)]
                dn = loop[(idx + 1) % len(loop)]
                return up, dn

            up_a, dn_a = upstream_downstream(loops[a_lid], a_idx)
            up_b, dn_b = upstream_downstream(loops[b_lid], b_idx)

            side_a = DiverterSide(side="a", c_coord=a_coord, loop_id=a_lid, upstream=up_a, downstream=dn_a)
            side_b = DiverterSide(side="b", c_coord=b_coord, loop_id=b_lid, upstream=up_b, downstream=dn_b)

            diverter_id = f"D{did:02d}"
            did += 1
            diverters[diverter_id] = Diverter(
                diverter_id=diverter_id,
                side_a=side_a,
                side_b=side_b,
                ticks_per_90=diverter_ticks_per_90,
                angle_deg=0.0,
                target_angle_deg=0.0,
            )

        return Topology(
            grid=grid,
            graph=graph,
            loops=loops,
            outer_loop_id=outer_loop_id,
            coord_to_loop_idx=coord_to_loop_idx,
            diverters=diverters,
        )
