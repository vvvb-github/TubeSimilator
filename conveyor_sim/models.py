from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple, Literal, Union, Any

Coord = Tuple[int, int]  # (row, col)

# =========================
# Domain entities
# =========================

@dataclass
class Tube:
    tube_id: str
    required_project: str
    completed_projects: Set[str] = field(default_factory=set)
    created_tick: int = 0

    @property
    def is_done(self) -> bool:
        return self.required_project in self.completed_projects


@dataclass
class Base:
    base_id: str
    coord: Optional[Coord]  # None when in diverter cargo
    tube_id: Optional[str] = None

    # Transfer state (visual + logic)
    in_transfer: Optional[str] = None  # diverter_id if being transferred
    transfer_dst: Optional[Coord] = None

    def is_empty(self) -> bool:
        return self.tube_id is None


@dataclass
class Station:
    station_id: str
    kind: Literal["S", "I"]
    coord: Coord
    projects: Set[str]
    interaction_ticks: int

    stop_diverter_id: str
    stop_diverter_side: Literal["a", "b"]

    busy_remaining: int = 0
    busy_base_id: Optional[str] = None
    busy_op: Optional[Literal["test", "load", "unload"]] = None

    def is_busy(self) -> bool:
        return self.busy_remaining > 0


@dataclass
class DiverterSide:
    side: Literal["a", "b"]
    c_coord: Coord
    loop_id: int
    upstream: Coord
    downstream: Coord


@dataclass
class Diverter:
    diverter_id: str
    side_a: DiverterSide
    side_b: DiverterSide
    ticks_per_90: int = 1

    # Busy state
    busy_remaining: int = 0
    cargo_base_id: Optional[str] = None
    cargo_dst: Optional[Coord] = None

    # For visualization (angle in degrees)
    angle_deg: float = 0.0
    target_angle_deg: float = 0.0
    rotating: bool = False

    def is_busy(self) -> bool:
        return self.busy_remaining > 0

    def side(self, s: Literal["a", "b"]) -> DiverterSide:
        return self.side_a if s == "a" else self.side_b


# =========================
# Actions (control commands)
# =========================

@dataclass(frozen=True)
class StationStart:
    station_id: str
    op: Literal["test", "load", "unload"]


@dataclass(frozen=True)
class DiverterTransfer:
    diverter_id: str
    src_side: Literal["a", "b"]
    dst_side: Literal["a", "b"]


Action = Union[StationStart, DiverterTransfer]


# =========================
# Snapshot views (scheduler uses)
# =========================

@dataclass(frozen=True)
class BaseView:
    base_id: str
    coord: Optional[Coord]
    tube_id: Optional[str]
    in_transfer: Optional[str]
    transfer_dst: Optional[Coord]


@dataclass(frozen=True)
class TubeView:
    tube_id: str
    required_project: str
    completed_projects: List[str]


@dataclass(frozen=True)
class StationView:
    station_id: str
    kind: str
    coord: Coord
    projects: List[str]
    interaction_ticks: int
    stop_diverter_id: str
    stop_diverter_side: str
    busy_remaining: int
    busy_base_id: Optional[str]
    busy_op: Optional[str]


@dataclass(frozen=True)
class DiverterSideView:
    side: str
    c_coord: Coord
    loop_id: int
    upstream: Coord
    downstream: Coord


@dataclass(frozen=True)
class DiverterView:
    diverter_id: str
    busy_remaining: int
    cargo_base_id: Optional[str]
    cargo_dst: Optional[Coord]
    angle_deg: float
    side_a: DiverterSideView
    side_b: DiverterSideView


@dataclass(frozen=True)
class Snapshot:
    tick: int
    outer_loop_id: int
    bases: List[BaseView]
    tubes: Dict[str, TubeView]
    stations: Dict[str, StationView]
    diverters: Dict[str, DiverterView]
    ready_input_tubes: List[str]
    alarms: List[str]
