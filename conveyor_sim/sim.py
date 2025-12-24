from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, Set, Iterable, Any
import random

from .config import Config
from .grid import Grid
from .topology import Topology
from .models import (
    Action,
    Base,
    Tube,
    Station,
    Diverter,
    StationStart,
    DiverterTransfer,
    Snapshot,
    BaseView,
    TubeView,
    StationView,
    DiverterView,
    DiverterSideView,
)

Coord = Tuple[int, int]


@dataclass
class FrameData:
    tick: int
    base_start_xy: Dict[str, Tuple[float, float]]
    base_end_xy: Dict[str, Tuple[float, float]]
    diverter_start_angle: Dict[str, float]
    diverter_end_angle: Dict[str, float]
    alarms: List[str]


class ConveyorSystem:
    """Tick-based simulator for the conveyor system.

    Core rules implemented:
    - Belt moves 1 cell per tick for any base that is not blocked.
    - A base is blocked if the cell ahead contains a diverter cell ('C') OR
      contains a base that is itself blocked (jam propagation).
    - Diverter ('C') cells are always blocked for belt motion; to pass them,
      the controller must issue a DiverterTransfer action, which 'teleports'
      a base from upstream to downstream (straight or lane-change).
    - Stations (S/I) can interact with a base ONLY when that base is stopped
      at the station interaction point (upstream of the configured stop diverter).
    """

    def __init__(
        self,
        grid: Grid,
        topo: Topology,
        bases: Dict[str, Base],
        tubes: Dict[str, Tube],
        stations: Dict[str, Station],
        diverters: Dict[str, Diverter],
        tube_arrivals: List[Tuple[int, str]],  # list of (time, tube_id)
    ) -> None:
        self.grid = grid
        self.topo = topo
        self.bases = bases
        self.tubes = tubes
        self.stations = stations
        self.diverters = diverters

        self.tube_arrivals = tube_arrivals
        self._arrival_idx = 0
        self.ready_input_tubes: List[str] = []

        self.completed_output: List[str] = []

        self.tick: int = 0
        self.alarms: List[str] = []

        # Precompute for speed
        self._coord_to_loop_idx = topo.coord_to_loop_idx
        self._loops = topo.loops
        self.outer_loop_id = topo.outer_loop_id

        # Map C coord -> (diverter_id, side)
        self._c_to_div: Dict[Coord, Tuple[str, str]] = {}
        for did, d in diverters.items():
            self._c_to_div[d.side_a.c_coord] = (did, "a")
            self._c_to_div[d.side_b.c_coord] = (did, "b")

        # Basic integrity checks
        self._validate_station_positions()

    # ---------------------------
    # Construction
    # ---------------------------

    @staticmethod
    def from_yaml(path: str) -> "ConveyorSystem":
        cfg = Config.from_yaml(path)
        grid = Grid.from_rows(cfg.grid.rows)
        topo = Topology.from_grid(grid, diverter_ticks_per_90=cfg.sim.diverter_ticks_per_90)

        # Map station coords
        s_coords = grid.coords_with("S")
        s_coords_sorted = sorted(s_coords)
        if len(s_coords_sorted) != len(cfg.stations):
            raise ValueError(f"Grid has {len(s_coords_sorted)} 'S' cells but config defines {len(cfg.stations)} stations.")

        # IO coord
        i_coords = grid.coords_with("I")
        if len(i_coords) != 1:
            raise ValueError(f"Grid must contain exactly one 'I'. Found {len(i_coords)}")
        i_coord = i_coords[0]

        # Create stations dict
        stations: Dict[str, Station] = {}

        # helper: find stop diverter for a station coord
        def stop_diverter_for(coord: Coord) -> Tuple[str, str]:
            if coord not in topo.coord_to_loop_idx:
                raise ValueError(f"Station coord {coord} is not on any loop.")
            lid, idx = topo.coord_to_loop_idx[coord]
            loop = topo.loops[lid]
            downstream = loop[(idx + 1) % len(loop)]
            if grid.get(downstream) != "C":
                raise ValueError(
                    f"Station at {coord} must have a downstream 'C' (stopper diverter).\n"
                    f"But downstream cell is {downstream} token='{grid.get(downstream)}'"
                )
            # find diverter id/side
            if downstream not in {d.side_a.c_coord for d in topo.diverters.values()} and downstream not in {d.side_b.c_coord for d in topo.diverters.values()}:
                raise ValueError(f"Downstream C at {downstream} is not part of any diverter pair.")
            # we'll build mapping from topo.diverters
            for did, dv in topo.diverters.items():
                if dv.side_a.c_coord == downstream:
                    return did, "a"
                if dv.side_b.c_coord == downstream:
                    return did, "b"
            raise ValueError("Unreachable")

        # Stations S
        for idx, scfg in enumerate(cfg.stations):
            coord = scfg.pos if scfg.pos is not None else s_coords_sorted[idx]
            did, side = stop_diverter_for(coord)
            stations[scfg.station_id] = Station(
                station_id=scfg.station_id,
                kind="S",
                coord=coord,
                projects=set(scfg.projects),
                interaction_ticks=int(scfg.interaction_ticks),
                stop_diverter_id=did,
                stop_diverter_side=side,
            )

        # IO station I
        io_did, io_side = stop_diverter_for(i_coord if cfg.io.pos is None else cfg.io.pos)
        stations[cfg.io.station_id] = Station(
            station_id=cfg.io.station_id,
            kind="I",
            coord=i_coord if cfg.io.pos is None else cfg.io.pos,
            projects=set(),
            interaction_ticks=int(cfg.io.interaction_ticks),
            stop_diverter_id=io_did,
            stop_diverter_side=io_side,
        )

        # Diverters come from topology but need updated ticks_per_90 from config
        diverters: Dict[str, Diverter] = topo.diverters
        for d in diverters.values():
            d.ticks_per_90 = cfg.sim.diverter_ticks_per_90

        # Tubes: create objects for each arrival
        tubes: Dict[str, Tube] = {}
        tube_arrivals: List[Tuple[int, str]] = []
        for a in cfg.tube_arrivals:
            if a.tube_id in tubes:
                raise ValueError(f"Duplicate tube_id in arrivals: {a.tube_id}")
            tubes[a.tube_id] = Tube(tube_id=a.tube_id, required_project=a.project, created_tick=a.time)
            tube_arrivals.append((a.time, a.tube_id))
        tube_arrivals.sort()

        # Bases: place randomly on non-C track cells
        rng = random.Random(cfg.sim.seed)
        candidate = [rc for rc in grid.iter_track_coords() if grid.get(rc) != "C"]
        if cfg.sim.base_count > len(candidate):
            raise ValueError(f"base_count={cfg.sim.base_count} exceeds available non-C track cells {len(candidate)}")
        rng.shuffle(candidate)
        bases: Dict[str, Base] = {}
        for i in range(cfg.sim.base_count):
            bid = f"B{i:02d}"
            bases[bid] = Base(base_id=bid, coord=candidate[i])

        return ConveyorSystem(
            grid=grid,
            topo=topo,
            bases=bases,
            tubes=tubes,
            stations=stations,
            diverters=diverters,
            tube_arrivals=tube_arrivals,
        )

    # ---------------------------
    # Public API
    # ---------------------------

    def snapshot(self) -> Snapshot:
        bases_view = [
            BaseView(
                base_id=b.base_id,
                coord=b.coord,
                tube_id=b.tube_id,
                in_transfer=b.in_transfer,
                transfer_dst=b.transfer_dst,
            )
            for b in self.bases.values()
        ]
        tubes_view: Dict[str, TubeView] = {
            tid: TubeView(
                tube_id=t.tube_id,
                required_project=t.required_project,
                completed_projects=sorted(list(t.completed_projects)),
            )
            for tid, t in self.tubes.items()
        }
        stations_view: Dict[str, StationView] = {
            sid: StationView(
                station_id=s.station_id,
                kind=s.kind,
                coord=s.coord,
                projects=sorted(list(s.projects)),
                interaction_ticks=s.interaction_ticks,
                stop_diverter_id=s.stop_diverter_id,
                stop_diverter_side=s.stop_diverter_side,
                busy_remaining=s.busy_remaining,
                busy_base_id=s.busy_base_id,
                busy_op=s.busy_op,
            )
            for sid, s in self.stations.items()
        }
        diverters_view: Dict[str, DiverterView] = {}
        for did, d in self.diverters.items():
            diverters_view[did] = DiverterView(
                diverter_id=did,
                busy_remaining=d.busy_remaining,
                cargo_base_id=d.cargo_base_id,
                cargo_dst=d.cargo_dst,
                angle_deg=d.angle_deg,
                side_a=DiverterSideView(
                    side="a",
                    c_coord=d.side_a.c_coord,
                    loop_id=d.side_a.loop_id,
                    upstream=d.side_a.upstream,
                    downstream=d.side_a.downstream,
                ),
                side_b=DiverterSideView(
                    side="b",
                    c_coord=d.side_b.c_coord,
                    loop_id=d.side_b.loop_id,
                    upstream=d.side_b.upstream,
                    downstream=d.side_b.downstream,
                ),
            )

        return Snapshot(
            tick=self.tick,
            outer_loop_id=self.outer_loop_id,
            bases=bases_view,
            tubes=tubes_view,
            stations=stations_view,
            diverters=diverters_view,
            ready_input_tubes=list(self.ready_input_tubes),
            alarms=list(self.alarms),
        )

    def step(self, actions: List[Action]) -> FrameData:
        self.alarms = []

        # 0) move arrivals to ready queue
        while self._arrival_idx < len(self.tube_arrivals) and self.tube_arrivals[self._arrival_idx][0] <= self.tick:
            _, tid = self.tube_arrivals[self._arrival_idx]
            self.ready_input_tubes.append(tid)
            self._arrival_idx += 1

        # 1) record start visuals
        base_start_xy = {bid: self._base_visual_xy(b) for bid, b in self.bases.items()}
        diverter_start_angle = {did: d.angle_deg for did, d in self.diverters.items()}

        # 2) apply actions (starts)
        self._apply_actions(actions)

        # 3) belt movement (1 cell if not blocked)
        self._belt_move()

        # 4) advance tasks (decrement + complete)
        self._advance_tasks()

        # 5) record end visuals
        base_end_xy = {bid: self._base_visual_xy(b) for bid, b in self.bases.items()}
        diverter_end_angle = {did: d.angle_deg for did, d in self.diverters.items()}

        frame = FrameData(
            tick=self.tick,
            base_start_xy=base_start_xy,
            base_end_xy=base_end_xy,
            diverter_start_angle=diverter_start_angle,
            diverter_end_angle=diverter_end_angle,
            alarms=list(self.alarms),
        )

        self.tick += 1
        return frame

    # ---------------------------
    # Internals
    # ---------------------------

    def _validate_station_positions(self) -> None:
        # Ensure each station's stop diverter is actually downstream
        for sid, st in self.stations.items():
            lid, idx = self._coord_to_loop_idx[st.coord]
            loop = self._loops[lid]
            downstream = loop[(idx + 1) % len(loop)]
            if self.grid.get(downstream) != "C":
                raise ValueError(f"Station {sid} at {st.coord} has no downstream C.")
            # Also ensure that C belongs to that diverter
            did, side = self._c_to_div.get(downstream, (None, None))  # type: ignore
            if did != st.stop_diverter_id:
                raise ValueError(f"Station {sid} stop diverter mismatch: config={st.stop_diverter_id}, inferred={did}")

    def _base_visual_xy(self, b: Base) -> Tuple[float, float]:
        # map cell center: x=col, y=-row
        if b.coord is not None:
            r, c = b.coord
            return (float(c), float(-r))
        # in diverter cargo: show at diverter center
        if b.in_transfer and b.in_transfer in self.diverters:
            d = self.diverters[b.in_transfer]
            (r1, c1) = d.side_a.c_coord
            (r2, c2) = d.side_b.c_coord
            return (float(c1 + c2) / 2.0, float(-(r1 + r2)) / 2.0)
        return (0.0, 0.0)

    def _apply_actions(self, actions: List[Action]) -> None:
        # Separate station/diverter actions. Apply station first so they can "claim" a base.
        for act in actions:
            if isinstance(act, StationStart):
                self._start_station(act)
        for act in actions:
            if isinstance(act, DiverterTransfer):
                self._start_diverter_transfer(act)

    def _start_station(self, act: StationStart) -> None:
        if act.station_id not in self.stations:
            self._alarm(f"[tick {self.tick}] Unknown station_id '{act.station_id}'")
            return
        st = self.stations[act.station_id]
        if st.is_busy():
            self._alarm(f"[tick {self.tick}] Station {st.station_id} already busy.")
            return

        base_id = self._base_at_coord(st.coord)
        if base_id is None:
            self._alarm(f"[tick {self.tick}] Station {st.station_id} start '{act.op}' but no base at station.")
            return
        b = self.bases[base_id]

        # Must be stopped at interaction point: downstream is stop-diverter C cell
        lid, idx = self._coord_to_loop_idx[st.coord]
        loop = self._loops[lid]
        downstream = loop[(idx + 1) % len(loop)]
        if downstream != self.diverters[st.stop_diverter_id].side(st.stop_diverter_side).c_coord:
            self._alarm(f"[tick {self.tick}] Station {st.station_id} downstream mismatch (stop diverter not immediately after station).")
            return

        # If the controller is transferring it away this tick, it won't be here. We enforce: cannot start if base already removed.
        if b.coord is None or b.in_transfer is not None:
            self._alarm(f"[tick {self.tick}] Station {st.station_id} cannot start; base {base_id} is in transfer.")
            return

        if act.op == "test":
            if st.kind != "S":
                self._alarm(f"[tick {self.tick}] Station {st.station_id} is kind={st.kind}, cannot 'test'.")
                return
            if b.tube_id is None:
                self._alarm(f"[tick {self.tick}] Station {st.station_id} test requested but base {base_id} is empty.")
                return
            tube = self.tubes[b.tube_id]
            if tube.required_project not in st.projects:
                self._alarm(f"[tick {self.tick}] Station {st.station_id} cannot do project {tube.required_project} for tube {tube.tube_id}.")
                return
            if tube.required_project in tube.completed_projects:
                self._alarm(f"[tick {self.tick}] Tube {tube.tube_id} already completed {tube.required_project}; redundant test.")
                # still allow (no-op) if you want; here we allow but warn.
            st.busy_remaining = st.interaction_ticks
            st.busy_base_id = base_id
            st.busy_op = "test"
            return

        if act.op == "load":
            if st.kind != "I":
                self._alarm(f"[tick {self.tick}] Station {st.station_id} is kind={st.kind}, cannot 'load'.")
                return
            if b.tube_id is not None:
                self._alarm(f"[tick {self.tick}] I station load requested but base {base_id} already has tube {b.tube_id}.")
                return
            if not self.ready_input_tubes:
                self._alarm(f"[tick {self.tick}] I station load requested but input buffer empty.")
                return
            st.busy_remaining = st.interaction_ticks
            st.busy_base_id = base_id
            st.busy_op = "load"
            return

        if act.op == "unload":
            if st.kind != "I":
                self._alarm(f"[tick {self.tick}] Station {st.station_id} is kind={st.kind}, cannot 'unload'.")
                return
            if b.tube_id is None:
                self._alarm(f"[tick {self.tick}] I station unload requested but base {base_id} is empty.")
                return
            tube = self.tubes[b.tube_id]
            if not tube.is_done:
                self._alarm(f"[tick {self.tick}] I station unload requested but tube {tube.tube_id} not done (needs {tube.required_project}).")
                return
            st.busy_remaining = st.interaction_ticks
            st.busy_base_id = base_id
            st.busy_op = "unload"
            return

        self._alarm(f"[tick {self.tick}] Unknown station op '{act.op}'")

    def _start_diverter_transfer(self, act: DiverterTransfer) -> None:
        if act.diverter_id not in self.diverters:
            self._alarm(f"[tick {self.tick}] Unknown diverter_id '{act.diverter_id}'")
            return
        dv = self.diverters[act.diverter_id]
        if dv.is_busy():
            self._alarm(f"[tick {self.tick}] Diverter {dv.diverter_id} busy; transfer ignored.")
            return

        src = dv.side(act.src_side)
        dst = dv.side(act.dst_side)

        base_id = self._base_at_coord(src.upstream)
        if base_id is None:
            self._alarm(f"[tick {self.tick}] Diverter {dv.diverter_id} transfer {act.src_side}->{act.dst_side} but no base at upstream {src.upstream}.")
            return

        # Prevent picking a base currently being processed at a station
        if any(st.busy_base_id == base_id for st in self.stations.values() if st.is_busy()):
            self._alarm(f"[tick {self.tick}] Diverter {dv.diverter_id} cannot transfer base {base_id}; it is being processed by a station.")
            return

        # Destination must be empty (and not a diverter cell)
        if self.grid.get(dst.downstream) == "C":
            self._alarm(f"[tick {self.tick}] Diverter {dv.diverter_id} dst downstream is a C cell?? {dst.downstream}")
            return
        if self._base_at_coord(dst.downstream) is not None:
            self._alarm(f"[tick {self.tick}] Diverter {dv.diverter_id} dst downstream {dst.downstream} occupied; transfer blocked.")
            return

        # Start transfer: remove base from track
        b = self.bases[base_id]
        b.coord = None
        b.in_transfer = dv.diverter_id
        b.transfer_dst = dst.downstream

        dv.cargo_base_id = base_id
        dv.cargo_dst = dst.downstream

        duration = self._diverter_duration_ticks(dv, act.src_side, act.dst_side)
        dv.busy_remaining = duration

        # Set angle target for visualization
        step = 90.0 / max(1, dv.ticks_per_90)
        dv.target_angle_deg = dv.angle_deg + step * duration
        dv.rotating = True

    def _diverter_duration_ticks(self, dv: Diverter, src_side: str, dst_side: str) -> int:
        # Simplified model aligned with the user's example:
        # - Straight (same side) : 90 deg
        # - Lane change (cross)  : 360 deg (two 180 deg rotations)
        if dv.ticks_per_90 <= 0:
            return 1
        if src_side == dst_side:
            return dv.ticks_per_90 * 1
        return dv.ticks_per_90 * 4

    def _advance_tasks(self) -> None:
        # Update diverter angle first (visual)
        for dv in self.diverters.values():
            if dv.rotating and dv.busy_remaining > 0:
                step = 90.0 / max(1, dv.ticks_per_90)
                dv.angle_deg += step
                if dv.angle_deg > dv.target_angle_deg:
                    dv.angle_deg = dv.target_angle_deg

        # Decrement busy and complete diverter transfers
        for dv in self.diverters.values():
            if dv.busy_remaining <= 0:
                dv.rotating = False
                continue
            dv.busy_remaining -= 1
            if dv.busy_remaining == 0:
                # complete: drop base
                if dv.cargo_base_id is None or dv.cargo_dst is None:
                    self._alarm(f"[tick {self.tick}] Diverter {dv.diverter_id} completed but missing cargo.")
                else:
                    bid = dv.cargo_base_id
                    dst = dv.cargo_dst
                    if self._base_at_coord(dst) is not None:
                        self._alarm(f"[tick {self.tick}] Diverter {dv.diverter_id} drop collision at {dst}")
                    else:
                        b = self.bases[bid]
                        b.coord = dst
                        b.in_transfer = None
                        b.transfer_dst = None
                dv.cargo_base_id = None
                dv.cargo_dst = None
                dv.rotating = False
                # snap angle
                dv.angle_deg = dv.target_angle_deg

        # Decrement busy and complete stations
        for st in self.stations.values():
            if st.busy_remaining <= 0:
                continue
            st.busy_remaining -= 1
            if st.busy_remaining == 0:
                # complete action
                if st.busy_base_id is None or st.busy_op is None:
                    self._alarm(f"[tick {self.tick}] Station {st.station_id} completed but missing context.")
                    st.busy_base_id = None
                    st.busy_op = None
                    continue
                b = self.bases[st.busy_base_id]
                if st.busy_op == "test":
                    if b.tube_id is None:
                        self._alarm(f"[tick {self.tick}] Station {st.station_id} test completed but base empty.")
                    else:
                        tube = self.tubes[b.tube_id]
                        if tube.required_project in st.projects:
                            tube.completed_projects.add(tube.required_project)
                elif st.busy_op == "load":
                    if b.tube_id is not None:
                        self._alarm(f"[tick {self.tick}] I load completed but base already has tube.")
                    elif not self.ready_input_tubes:
                        self._alarm(f"[tick {self.tick}] I load completed but input buffer empty.")
                    else:
                        tid = self.ready_input_tubes.pop(0)
                        b.tube_id = tid
                elif st.busy_op == "unload":
                    if b.tube_id is None:
                        self._alarm(f"[tick {self.tick}] I unload completed but base empty.")
                    else:
                        tid = b.tube_id
                        tube = self.tubes[tid]
                        if tube.is_done:
                            b.tube_id = None
                            self.completed_output.append(tid)
                        else:
                            self._alarm(f"[tick {self.tick}] I unload completed but tube not done.")
                else:
                    self._alarm(f"[tick {self.tick}] Unknown busy_op {st.busy_op}")
                st.busy_base_id = None
                st.busy_op = None

    def _belt_move(self) -> None:
        # Build occupancy per loop
        # coord -> base_id for bases on track
        coord_occ: Dict[Coord, str] = {}
        for bid, b in self.bases.items():
            if b.coord is not None:
                if b.coord in coord_occ:
                    self._alarm(f"[tick {self.tick}] Collision: two bases on {b.coord}")
                coord_occ[b.coord] = bid

        for lid, loop in enumerate(self._loops):
            L = len(loop)
            occ: List[Optional[str]] = [None] * L
            token: List[str] = [" "] * L
            for idx, rc in enumerate(loop):
                token[idx] = self.grid.get(rc)
                if rc in coord_occ:
                    occ[idx] = coord_occ[rc]

            # Determine blocked bases due to diverter cells and jam propagation
            blocked: Set[str] = set()
            for idx, bid in enumerate(occ):
                if bid is None:
                    continue
                nxt = (idx + 1) % L
                if token[nxt] == "C":
                    blocked.add(bid)

            changed = True
            while changed:
                changed = False
                for idx, bid in enumerate(occ):
                    if bid is None or bid in blocked:
                        continue
                    nxt = (idx + 1) % L
                    nxt_bid = occ[nxt]
                    if nxt_bid is not None and nxt_bid in blocked:
                        blocked.add(bid)
                        changed = True

            # Apply moves
            new_occ: List[Optional[str]] = [None] * L
            for idx, bid in enumerate(occ):
                if bid is None:
                    continue
                if bid in blocked:
                    new_occ[idx] = bid
                else:
                    nxt = (idx + 1) % L
                    if token[nxt] == "C":
                        # should not happen
                        new_occ[idx] = bid
                        continue
                    if new_occ[nxt] is not None:
                        self._alarm(f"[tick {self.tick}] Move collision into idx {nxt} on loop {lid}")
                        new_occ[idx] = bid
                    else:
                        new_occ[nxt] = bid

            # Commit back to bases
            for idx, bid in enumerate(new_occ):
                if bid is None:
                    continue
                self.bases[bid].coord = loop[idx]

    def _base_at_coord(self, coord: Coord) -> Optional[str]:
        for bid, b in self.bases.items():
            if b.coord == coord:
                return bid
        return None

    def _alarm(self, msg: str) -> None:
        self.alarms.append(msg)
