from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Set, Any

from .models import Snapshot, Action, StationStart, DiverterTransfer, Coord


@dataclass
class SimpleScheduler:
    """A simple greedy scheduler.

    Strategy:
    - Keep bases moving by "straight-pass" through every diverter by default.
    - Stop a base at I when:
        * it is empty and there is a ready tube to load, or
        * it carries a completed tube to unload.
    - Stop a base at S when it carries a tube requiring that station's project and not done.
    - Perform station operations whenever idle and the right base is present.
    """

    def decide(self, snap: Snapshot) -> List[Action]:
        actions: List[Action] = []

        # Build fast lookup
        coord_to_base: Dict[Coord, str] = {}
        base_tube: Dict[str, Optional[str]] = {}
        base_transfer: Dict[str, Optional[str]] = {}
        for b in snap.bases:
            if b.coord is not None:
                coord_to_base[b.coord] = b.base_id
            base_tube[b.base_id] = b.tube_id
            base_transfer[b.base_id] = b.in_transfer

        # Identify IO + S stations
        io_id = None
        s_ids: List[str] = []
        for sid, st in snap.stations.items():
            if st.kind == "I":
                io_id = sid
            elif st.kind == "S":
                s_ids.append(sid)
        if io_id is None:
            return actions

        # Stations we need to hold (do not transfer away) this tick
        hold_coords: Set[Coord] = set()

        # ---- Station actions ----
        # IO station
        io = snap.stations[io_id]
        if io.busy_remaining == 0:
            bid = coord_to_base.get(io.coord)
            if bid is not None and base_transfer.get(bid) is None:
                tid = base_tube.get(bid)
                if tid is not None:
                    tube = snap.tubes[tid]
                    if tube.required_project in tube.completed_projects:
                        actions.append(StationStart(station_id=io_id, op="unload"))
                        hold_coords.add(io.coord)
                else:
                    if len(snap.ready_input_tubes) > 0:
                        actions.append(StationStart(station_id=io_id, op="load"))
                        hold_coords.add(io.coord)
        else:
            # busy => must hold
            hold_coords.add(io.coord)

        # S stations
        for sid in s_ids:
            st = snap.stations[sid]
            if st.busy_remaining > 0:
                hold_coords.add(st.coord)
                continue
            bid = coord_to_base.get(st.coord)
            if bid is None or base_transfer.get(bid) is not None:
                continue
            tid = base_tube.get(bid)
            if tid is None:
                continue
            tube = snap.tubes[tid]
            if tube.required_project in tube.completed_projects:
                continue
            if tube.required_project in st.projects:
                actions.append(StationStart(station_id=sid, op="test"))
                hold_coords.add(st.coord)

        # ---- Diverter transfers ----
        # Mapping stop diverter -> station coord to hold
        stop_diverter_hold: Set[Tuple[str, Coord]] = set()
        for sid, st in snap.stations.items():
            if st.coord in hold_coords:
                stop_diverter_hold.add((st.stop_diverter_id, st.coord))

        for did, dv in snap.diverters.items():
            if dv.busy_remaining > 0:
                continue

            # side priority: outer loop first
            sides = [dv.side_a, dv.side_b]
            sides.sort(key=lambda s: 0 if s.loop_id == snap.outer_loop_id else 1)

            chosen = None
            for side in sides:
                bid = coord_to_base.get(side.upstream)
                if bid is None:
                    continue
                # If this upstream is a station interaction point we must hold, skip
                if (did, side.upstream) in stop_diverter_hold:
                    continue
                chosen = side
                break

            if chosen is not None:
                # Destination must be empty, otherwise don't issue transfer this tick
                if chosen.downstream in coord_to_base:
                    continue
                actions.append(DiverterTransfer(diverter_id=did, src_side=chosen.side, dst_side=chosen.side))

        return actions


# --------------------------
# Advanced / buffering scheduler
# --------------------------


def _other_side(side: str) -> str:
    return "b" if side == "a" else "a"


@dataclass(frozen=True)
class _ResolvedDiverter:
    """A diverter resolved from an (outer-loop) C coordinate."""

    diverter_id: str
    outer_side: str  # 'a' or 'b'
    inner_side: str  # the other side


@dataclass
class BufferingScheduler:
    """Scheduler that demonstrates *actual* lane-change + buffering.

    Concept:
    - Inner loop is treated as a fast lane: keep it flowing by prioritizing its straight passes.
    - Outer loop is used as a set of category buffers (A / B / I) by deliberately blocking
      specific diverters (buffer "gates").
    - When a station queue has space, release a base from the corresponding buffer to the inner
      loop (fast lane), then later merge it back to the outer loop *downstream of upstream stations*
      so that each station can be supplied without being blocked by upstream queues.

    This scheduler relies on a small amount of routing metadata (buffer gate/entry and merge points).
    For the demo, this is provided in the YAML under the `advanced_scheduler` section.
    """

    system: Any
    plan: Dict[str, Any]

    # Tunables
    inner_max_bases: int = 25
    prefer_lane_change_out_of_station: bool = True
    prioritize_inner_straight: bool = True
    isolate_station_segments: bool = True

    # Derived mappings
    _initialized: bool = field(default=False, init=False)
    _io_station_id: str = field(default="I", init=False)
    _outer_loop_id: int = field(default=0, init=False)
    _project_to_station: Dict[str, str] = field(default_factory=dict, init=False)
    _queue_capacity: Dict[str, int] = field(default_factory=dict, init=False)
    _queue_slots: Dict[str, List[Coord]] = field(default_factory=dict, init=False)
    _merge: Dict[str, _ResolvedDiverter] = field(default_factory=dict, init=False)
    _buffer_entry: Dict[str, _ResolvedDiverter] = field(default_factory=dict, init=False)
    _buffer_gate: Dict[str, _ResolvedDiverter] = field(default_factory=dict, init=False)

    def _resolve_outer_c(self, outer_c: Coord) -> _ResolvedDiverter:
        """Map an outer-loop C coordinate to a diverter id and its outer/inner sides."""
        for did, dv in self.system.diverters.items():
            if dv.side_a.c_coord == outer_c:
                side = "a"
            elif dv.side_b.c_coord == outer_c:
                side = "b"
            else:
                continue

            outer_side = side
            inner_side = _other_side(side)
            outer_loop_id = getattr(dv, f"side_{outer_side}").loop_id
            inner_loop_id = getattr(dv, f"side_{inner_side}").loop_id
            if outer_loop_id != self.system.outer_loop_id:
                raise ValueError(
                    f"advanced_scheduler: expected outer_c={outer_c} to be on outer loop, "
                    f"but {did}.{outer_side} is on loop {outer_loop_id} (outer={self.system.outer_loop_id})."
                )
            if inner_loop_id == outer_loop_id:
                raise ValueError(
                    f"advanced_scheduler: expected diverter {did} to connect outer<->inner, "
                    f"but both sides are on loop {outer_loop_id}."
                )
            return _ResolvedDiverter(diverter_id=did, outer_side=outer_side, inner_side=inner_side)

        raise KeyError(f"advanced_scheduler: no diverter found with outer C at {outer_c}.")

    def _init_if_needed(self, snap: Snapshot) -> None:
        if self._initialized:
            return

        self._outer_loop_id = self.system.outer_loop_id

        # Station lookup: required_project -> station_id
        self._io_station_id = next((sid for sid, st in snap.stations.items() if st.kind == "I"), "I")
        for sid, st in snap.stations.items():
            if st.kind != "S":
                continue
            for p in st.projects:
                self._project_to_station[p] = sid

        # Tunables from plan
        self.inner_max_bases = int(self.plan.get("inner_max_bases", self.inner_max_bases))
        self.prefer_lane_change_out_of_station = bool(
            self.plan.get("prefer_lane_change_out_of_station", self.prefer_lane_change_out_of_station)
        )
        self.prioritize_inner_straight = bool(self.plan.get("prioritize_inner_straight", self.prioritize_inner_straight))
        self.isolate_station_segments = bool(self.plan.get("isolate_station_segments", self.isolate_station_segments))

        # Queue capacities
        qc = self.plan.get("queue_capacity", {})
        if not isinstance(qc, dict):
            raise ValueError("advanced_scheduler.queue_capacity must be a mapping")
        self._queue_capacity = {str(k): int(v) for k, v in qc.items()}

        # Merge points
        merges = self.plan.get("merges", {})
        if not isinstance(merges, dict):
            raise ValueError("advanced_scheduler.merges must be a mapping")
        for station_id, m in merges.items():
            outer_c = tuple(m["outer_c"])  # type: ignore[arg-type]
            self._merge[str(station_id)] = self._resolve_outer_c(outer_c)  # type: ignore[arg-type]

        # Buffers
        buffers = self.plan.get("buffers", {})
        if not isinstance(buffers, dict):
            raise ValueError("advanced_scheduler.buffers must be a mapping")
        for station_id, b in buffers.items():
            entry_c = tuple(b["entry_outer_c"])  # type: ignore[arg-type]
            gate_c = tuple(b["gate_outer_c"])  # type: ignore[arg-type]
            self._buffer_entry[str(station_id)] = self._resolve_outer_c(entry_c)  # type: ignore[arg-type]
            self._buffer_gate[str(station_id)] = self._resolve_outer_c(gate_c)  # type: ignore[arg-type]

        # Pre-compute queue slot coordinates on the outer loop (skip C cells).
        outer_loop = self.system.topo.loops[self.system.outer_loop_id]
        for station_id, cap in self._queue_capacity.items():
            if station_id not in snap.stations:
                # allow config to omit some stations
                continue
            st_coord = snap.stations[station_id].coord
            # Find index of station coord on outer loop
            try:
                idx = outer_loop.index(st_coord)
            except ValueError as e:
                raise ValueError(f"Station {station_id} coord {st_coord} not on outer loop") from e

            slots: List[Coord] = []
            step = 0
            while len(slots) < cap and step < len(outer_loop) + 5:
                rc = outer_loop[(idx - step) % len(outer_loop)]
                if self.system.grid.get(rc) != "C":
                    slots.append(rc)
                step += 1
            if len(slots) < cap:
                raise ValueError(
                    f"Could not build {cap} queue slots for station {station_id}; got {len(slots)}"
                )
            self._queue_slots[station_id] = slots

        self._initialized = True

    def decide(self, snap: Snapshot) -> List[Action]:
        self._init_if_needed(snap)
        actions: List[Action] = []

        # ---- Fast lookup ----
        coord_to_base: Dict[Coord, str] = {}
        base_tube: Dict[str, Optional[str]] = {}
        base_transfer: Dict[str, Optional[str]] = {}
        base_coord: Dict[str, Optional[Coord]] = {}
        for b in snap.bases:
            base_coord[b.base_id] = b.coord
            if b.coord is not None:
                coord_to_base[b.coord] = b.base_id
            base_tube[b.base_id] = b.tube_id
            base_transfer[b.base_id] = b.in_transfer

        # Count bases on inner loop (fast lane pressure)
        inner_count = 0
        for bid, rc in base_coord.items():
            if rc is None:
                continue
            loop_id, _ = self.system.topo.coord_to_loop_idx[rc]
            if loop_id != self._outer_loop_id:
                inner_count += 1

        # Base -> target station id (S_A / S_B / I)
        def target_station_id_for_base(bid: str) -> str:
            tid = base_tube.get(bid)
            if tid is None:
                return self._io_station_id
            tube = snap.tubes[tid]
            if tube.required_project in tube.completed_projects:
                return self._io_station_id
            # Project -> station
            return self._project_to_station.get(tube.required_project, self._io_station_id)

        base_target_station: Dict[str, str] = {b.base_id: target_station_id_for_base(b.base_id) for b in snap.bases}

        # ---- Station actions + holds ----
        hold_coords: Set[Coord] = set()

        # IO station
        io_id = self._io_station_id
        if io_id in snap.stations:
            io = snap.stations[io_id]
            if io.busy_remaining == 0:
                bid = coord_to_base.get(io.coord)
                if bid is not None and base_transfer.get(bid) is None:
                    tid = base_tube.get(bid)
                    if tid is not None:
                        tube = snap.tubes[tid]
                        if tube.required_project in tube.completed_projects:
                            actions.append(StationStart(station_id=io_id, op="unload"))
                            hold_coords.add(io.coord)
                    else:
                        if len(snap.ready_input_tubes) > 0:
                            actions.append(StationStart(station_id=io_id, op="load"))
                            hold_coords.add(io.coord)
            else:
                hold_coords.add(io.coord)

        # S stations
        for sid, st in snap.stations.items():
            if st.kind != "S":
                continue
            if st.busy_remaining > 0:
                hold_coords.add(st.coord)
                continue
            bid = coord_to_base.get(st.coord)
            if bid is None or base_transfer.get(bid) is not None:
                continue
            tid = base_tube.get(bid)
            if tid is None:
                continue
            tube = snap.tubes[tid]
            if tube.required_project in tube.completed_projects:
                continue
            if tube.required_project in st.projects:
                actions.append(StationStart(station_id=sid, op="test"))
                hold_coords.add(st.coord)

        # Stop diverter -> station coord that must be held
        stop_diverter_hold: Set[Tuple[str, Coord]] = set()
        for sid, st in snap.stations.items():
            if st.coord in hold_coords:
                stop_diverter_hold.add((st.stop_diverter_id, st.coord))

        # ---- Queue availability ----
        queue_free: Dict[str, int] = {}
        for sid, slots in self._queue_slots.items():
            used = 0
            for rc in slots:
                if rc in coord_to_base:
                    used += 1
                else:
                    break
            queue_free[sid] = max(0, self._queue_capacity[sid] - used)

        used_diverters: Set[str] = set()

        def set_diverter_action(did: str, action: Action) -> None:
            if did in used_diverters:
                return
            dv = snap.diverters[did]
            if dv.busy_remaining > 0:
                return
            actions.append(action)
            used_diverters.add(did)

        def side_view(dv, side: str):
            return dv.side_a if side == "a" else dv.side_b

        buffer_gate_ids = {v.diverter_id for v in self._buffer_gate.values()}
        merge_ids = {v.diverter_id for v in self._merge.values()}

        # 1) Release from buffers (outer -> inner) when station queue has space
        for station_id, gate in self._buffer_gate.items():
            if queue_free.get(station_id, 0) <= 0:
                continue
            did = gate.diverter_id
            dv = snap.diverters[did]
            if dv.busy_remaining > 0 or did in used_diverters:
                continue
            o = side_view(dv, gate.outer_side)
            i = side_view(dv, gate.inner_side)
            bid = coord_to_base.get(o.upstream)
            if bid is None:
                continue
            if base_target_station.get(bid) != station_id:
                continue
            if i.downstream in coord_to_base:
                continue
            set_diverter_action(did, DiverterTransfer(diverter_id=did, src_side=gate.outer_side, dst_side=gate.inner_side))

        # 2) Merge from inner -> outer (downstream of upstream stations) when queue has space
        for station_id, m in self._merge.items():
            if queue_free.get(station_id, 0) <= 0:
                continue
            did = m.diverter_id
            dv = snap.diverters[did]
            if dv.busy_remaining > 0 or did in used_diverters:
                continue
            inner = side_view(dv, m.inner_side)
            outer = side_view(dv, m.outer_side)
            bid = coord_to_base.get(inner.upstream)
            if bid is None:
                continue
            if base_target_station.get(bid) != station_id:
                continue
            if outer.downstream in coord_to_base:
                continue
            set_diverter_action(did, DiverterTransfer(diverter_id=did, src_side=m.inner_side, dst_side=m.outer_side))

        # 3) After a station interaction (or if wrong base arrived), prefer lane-change out to inner
        if self.prefer_lane_change_out_of_station:
            for sid, st in snap.stations.items():
                if (st.stop_diverter_id, st.coord) in stop_diverter_hold:
                    continue
                bid = coord_to_base.get(st.coord)
                if bid is None:
                    continue
                # If station is idle and we're not holding, try to send the base to inner
                did = st.stop_diverter_id
                if did in used_diverters:
                    continue
                dv = snap.diverters[did]
                if dv.busy_remaining > 0:
                    continue
                outer_side = st.stop_diverter_side
                inner_side = _other_side(outer_side)
                inner_dn = side_view(dv, inner_side).downstream
                if inner_dn in coord_to_base:
                    continue
                set_diverter_action(did, DiverterTransfer(diverter_id=did, src_side=outer_side, dst_side=inner_side))

        # 4) If queue is full (or inner is too crowded), push bases into category buffers
        for station_id, entry in self._buffer_entry.items():
            want_buffer = (queue_free.get(station_id, 0) <= 0) or (inner_count > self.inner_max_bases)
            if not want_buffer:
                continue
            did = entry.diverter_id
            dv = snap.diverters[did]
            if dv.busy_remaining > 0 or did in used_diverters:
                continue
            inner = side_view(dv, entry.inner_side)
            outer = side_view(dv, entry.outer_side)
            bid = coord_to_base.get(inner.upstream)
            if bid is None:
                continue
            if base_target_station.get(bid) != station_id:
                continue
            if outer.downstream in coord_to_base:
                continue
            set_diverter_action(did, DiverterTransfer(diverter_id=did, src_side=entry.inner_side, dst_side=entry.outer_side))

        # 5) Default straight passes to keep things flowing
        for did, dv in snap.diverters.items():
            if dv.busy_remaining > 0 or did in used_diverters:
                continue

            sides = [dv.side_a, dv.side_b]
            if self.prioritize_inner_straight:
                sides.sort(key=lambda s: 0 if s.loop_id != snap.outer_loop_id else 1)
            else:
                sides.sort(key=lambda s: 0 if s.loop_id == snap.outer_loop_id else 1)

            for s in sides:
                bid = coord_to_base.get(s.upstream)
                if bid is None:
                    continue

                # Hold station bases if needed
                if (did, s.upstream) in stop_diverter_hold:
                    continue

                # Keep buffer gates blocking on outer side unless we already released above
                if did in buffer_gate_ids:
                    # Find gate mapping (if any) and suppress its outer side
                    for g in self._buffer_gate.values():
                        if g.diverter_id == did and s.side == g.outer_side:
                            bid2 = coord_to_base.get(s.upstream)
                            if bid2 is not None and base_target_station.get(bid2) in self._buffer_gate:
                                # Don't let buffered traffic pass the gate
                                s = None  # type: ignore
                            break
                    if s is None:
                        continue

                # Optionally isolate station segments by blocking outer straight across merge gates
                if self.isolate_station_segments and did in merge_ids:
                    for m in self._merge.values():
                        if m.diverter_id == did and s.side == m.outer_side:
                            # Keep the segment boundary closed (station segment isolation)
                            s = None  # type: ignore
                            break
                    if s is None:
                        continue

                if s.downstream in coord_to_base:
                    continue

                set_diverter_action(did, DiverterTransfer(diverter_id=did, src_side=s.side, dst_side=s.side))
                break

        return actions
