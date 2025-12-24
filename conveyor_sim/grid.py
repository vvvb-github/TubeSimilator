from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple, Set

Coord = Tuple[int, int]  # (row, col)

TRACK_TOKENS = {"=", "|", "C", "S", "I"}

@dataclass
class Grid:
    cells: List[List[str]]  # [row][col]

    @property
    def height(self) -> int:
        return len(self.cells)

    @property
    def width(self) -> int:
        return len(self.cells[0]) if self.cells else 0

    def in_bounds(self, rc: Coord) -> bool:
        r, c = rc
        return 0 <= r < self.height and 0 <= c < self.width

    def get(self, rc: Coord) -> str:
        r, c = rc
        return self.cells[r][c]

    def is_track(self, rc: Coord) -> bool:
        return self.get(rc) in TRACK_TOKENS

    def coords_with(self, token: str) -> List[Coord]:
        out: List[Coord] = []
        for r in range(self.height):
            for c in range(self.width):
                if self.cells[r][c] == token:
                    out.append((r, c))
        return out

    def iter_track_coords(self) -> Iterable[Coord]:
        for r in range(self.height):
            for c in range(self.width):
                if self.cells[r][c] in TRACK_TOKENS:
                    yield (r, c)

    @staticmethod
    def from_rows(rows: List[str]) -> "Grid":
        parsed: List[List[str]] = []
        maxw = 0
        for raw in rows:
            raw = raw.rstrip("\n")
            if isinstance(raw, str):
                # If row contains spaces, treat as token-separated.
                if " " in raw.strip():
                    toks = [t for t in raw.strip().split(" ") if t != ""]
                else:
                    toks = list(raw.strip())
            else:
                raise TypeError("Grid row must be string.")
            parsed.append(toks)
            maxw = max(maxw, len(toks))

        # pad to rectangle with spaces
        for row in parsed:
            if len(row) < maxw:
                row.extend([" "] * (maxw - len(row)))

        return Grid(cells=parsed)
