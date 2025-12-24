from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple, Set
import yaml

Coord = Tuple[int, int]

@dataclass
class StationConfig:
    station_id: str
    kind: str  # 'S'
    projects: Set[str]
    interaction_ticks: int
    pos: Optional[Coord] = None  # optional explicit coordinate


@dataclass
class IOConfig:
    station_id: str
    interaction_ticks: int
    pos: Optional[Coord] = None


@dataclass
class TubeArrivalConfig:
    time: int
    tube_id: str
    project: str


@dataclass
class SimConfig:
    base_count: int = 10
    seed: int = 0
    diverter_ticks_per_90: int = 1


@dataclass
class GridConfig:
    rows: List[str]


@dataclass
class Config:
    grid: GridConfig
    sim: SimConfig
    stations: List[StationConfig]
    io: IOConfig
    tube_arrivals: List[TubeArrivalConfig]

    @staticmethod
    def from_yaml(path: str) -> "Config":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        grid_rows = data["grid"]["rows"]

        sim_data = data.get("simulation", {})
        sim = SimConfig(
            base_count=int(sim_data.get("base_count", sim_data.get("bases", {}).get("count", 10))),
            seed=int(sim_data.get("seed", sim_data.get("bases", {}).get("seed", 0))),
            diverter_ticks_per_90=int(sim_data.get("diverter_ticks_per_90", 1)),
        )

        stations: List[StationConfig] = []
        for s in data.get("stations", []):
            stations.append(
                StationConfig(
                    station_id=str(s["id"]),
                    kind=str(s.get("kind", "S")),
                    projects=set(map(str, s.get("projects", []))),
                    interaction_ticks=int(s.get("interaction_ticks", 3)),
                    pos=tuple(s["pos"]) if "pos" in s and s["pos"] is not None else None,
                )
            )

        io_data = data.get("io", {})
        io = IOConfig(
            station_id=str(io_data.get("id", "I")),
            interaction_ticks=int(io_data.get("interaction_ticks", 5)),
            pos=tuple(io_data["pos"]) if "pos" in io_data and io_data["pos"] is not None else None,
        )

        arrivals: List[TubeArrivalConfig] = []
        for a in data.get("tube_arrivals", []):
            arrivals.append(
                TubeArrivalConfig(
                    time=int(a["time"]),
                    tube_id=str(a["tube_id"]),
                    project=str(a["project"]),
                )
            )
        arrivals.sort(key=lambda x: (x.time, x.tube_id))

        return Config(
            grid=GridConfig(rows=grid_rows),
            sim=sim,
            stations=stations,
            io=io,
            tube_arrivals=arrivals,
        )
