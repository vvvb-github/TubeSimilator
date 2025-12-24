from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional, Protocol
import math

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from .sim import ConveyorSystem, FrameData
from .models import Snapshot, Action


class Scheduler(Protocol):
    def decide(self, snap: Snapshot) -> List[Action]: ...


def _cell_xy(coord: Tuple[int, int]) -> Tuple[float, float]:
    r, c = coord
    return (float(c), float(-r))


def run_visualization(
    system: ConveyorSystem,
    scheduler: Scheduler,
    max_ticks: int = 500,
    subframes: int = 8,
    interval_ms: int = 50,
) -> None:
    """Run a realtime matplotlib animation.

    - subframes: how many animation frames per simulation tick.
    - interval_ms: milliseconds per animation frame.
    """
    grid = system.grid
    loops = system.topo.loops

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.set_aspect("equal")
    ax.set_title("Conveyor Simulator (tick-based)")
    ax.set_xlim(-1, grid.width)
    ax.set_ylim(-grid.height, 1)
    ax.axis("off")

    # Draw tracks as thick polylines
    for loop in loops:
        pts = [_cell_xy(rc) for rc in loop]
        xs = [p[0] for p in pts] + [pts[0][0]]
        ys = [p[1] for p in pts] + [pts[0][1]]
        ax.plot(xs, ys, linewidth=6, alpha=0.25)

    # Draw diverter cells
    c_coords = grid.coords_with("C")
    cx = [ _cell_xy(rc)[0] for rc in c_coords ]
    cy = [ _cell_xy(rc)[1] for rc in c_coords ]
    ax.scatter(cx, cy, s=300, marker="o", facecolors="none", edgecolors="black", linewidths=1.0)
    for rc in c_coords:
        x, y = _cell_xy(rc)
        ax.text(x, y, "C", ha="center", va="center", fontsize=10)

    # Draw stations
    for sid, st in system.stations.items():
        x, y = _cell_xy(st.coord)
        if st.kind == "I":
            ax.scatter([x], [y], s=500, marker="D")
        else:
            ax.scatter([x], [y], s=500, marker="s")
        ax.text(x, y + 0.3, sid, ha="center", va="bottom", fontsize=9)

    # Base scatter (updated each frame)
    base_ids = sorted(system.bases.keys())
    base_xy0 = [system._base_visual_xy(system.bases[bid]) for bid in base_ids]  # type: ignore
    base_sc = ax.scatter([p[0] for p in base_xy0], [p[1] for p in base_xy0], s=200, marker="o")

    # Belt markers
    marker_count_per_loop = 20
    belt_markers: List[float] = []  # one marker param per marker, global list
    belt_marker_loop: List[int] = []  # which loop
    for lid, loop in enumerate(loops):
        L = len(loop)
        for k in range(marker_count_per_loop):
            belt_markers.append(k * (L / marker_count_per_loop))
            belt_marker_loop.append(lid)
    belt_sc = ax.scatter([], [], s=10, marker=".", alpha=0.9)

    # Diverter rotating indicators (small line at diverter center)
    div_line: Dict[str, any] = {}
    div_vis_angle: Dict[str, float] = {}
    for did, dv in system.diverters.items():
        # center between two C cells
        x1, y1 = _cell_xy(dv.side_a.c_coord)
        x2, y2 = _cell_xy(dv.side_b.c_coord)
        cx0, cy0 = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        ln, = ax.plot([cx0, cx0 + 0.4], [cy0, cy0], linewidth=2)
        div_line[did] = (ln, cx0, cy0)
        div_vis_angle[did] = dv.angle_deg

    alarm_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, ha="left", va="top", fontsize=9)

    # Animation state
    current_frame: Optional[FrameData] = None
    current_tick: int = 0

    def compute_belt_marker_xy() -> Tuple[List[float], List[float]]:
        xs: List[float] = []
        ys: List[float] = []
        for i, s in enumerate(belt_markers):
            lid = belt_marker_loop[i]
            loop = loops[lid]
            L = len(loop)
            s_mod = s % L
            i0 = int(math.floor(s_mod))
            frac = s_mod - i0
            p0 = _cell_xy(loop[i0])
            p1 = _cell_xy(loop[(i0 + 1) % L])
            xs.append(p0[0] + frac * (p1[0] - p0[0]))
            ys.append(p0[1] + frac * (p1[1] - p0[1]))
        return xs, ys

    def update(frame_idx: int):
        nonlocal current_frame, current_tick

        tick_idx = frame_idx // subframes
        sub = frame_idx % subframes

        # Run sim step at the first subframe of each tick
        if sub == 0:
            if tick_idx >= max_ticks:
                return (base_sc, belt_sc, alarm_text)
            snap = system.snapshot()
            acts = scheduler.decide(snap)
            current_frame = system.step(acts)
            current_tick = current_frame.tick

        if current_frame is None:
            return (base_sc, belt_sc, alarm_text)

        alpha = (sub + 1) / subframes

        # Update bases + build a quick lookup of the current animated base positions.
        xs: List[float] = []
        ys: List[float] = []
        base_pos: Dict[str, Tuple[float, float]] = {}
        for bid in base_ids:
            x0, y0 = current_frame.base_start_xy[bid]
            x1, y1 = current_frame.base_end_xy[bid]
            x = x0 + alpha * (x1 - x0)
            y = y0 + alpha * (y1 - y0)
            xs.append(x)
            ys.append(y)
            base_pos[bid] = (x, y)
        base_sc.set_offsets(list(zip(xs, ys)))

        # Build a mapping from coordinate -> base_id for bases currently on track.
        coord_to_base: Dict[Tuple[int, int], str] = {}
        for bid, b in system.bases.items():
            if b.coord is None:
                continue
            coord_to_base[b.coord] = bid

        def _wrap_angle_deg(a: float) -> float:
            """Normalize angle to [-180, 180)."""
            a = (a + 180.0) % 360.0 - 180.0
            return a

        def _step_towards(current: float, target: float, max_step: float) -> float:
            diff = _wrap_angle_deg(target - current)
            if diff > max_step:
                diff = max_step
            elif diff < -max_step:
                diff = -max_step
            return current + diff

        # Update diverter lines
        for did, dv in system.diverters.items():
            ln, cx0, cy0 = div_line[did]

            # Decide what the diverter "should" aim at.
            focus_xy: Optional[Tuple[float, float]] = None

            # If the diverter is currently carrying a base, aim at the base while it is moving,
            # and aim at the *destination slot* while the base is held at the center.
            if dv.cargo_base_id is not None:
                bid = dv.cargo_base_id
                bxy = base_pos.get(bid)
                if bxy is not None:
                    # When the base is exactly at the diverter center (held phase),
                    # rotate towards the destination slot.
                    #
                    # NOTE: the simulator stores the destination as an absolute grid
                    # coordinate (cargo_dst), not as a side label.
                    if abs(bxy[0] - cx0) + abs(bxy[1] - cy0) < 1e-6 and dv.cargo_dst is not None:
                        focus_xy = _cell_xy(dv.cargo_dst)
                    else:
                        focus_xy = bxy

            # Otherwise, idle: aim at the next waiting base (prefer inner side) so the line
            # turns *before* the base is moved.
            if focus_xy is None:
                # pick an upstream base: inner first
                sides = [dv.side_a, dv.side_b]
                sides.sort(key=lambda s: 0 if s.loop_id != system.outer_loop_id else 1)
                chosen_bid: Optional[str] = None
                for s in sides:
                    if s.upstream in coord_to_base:
                        chosen_bid = coord_to_base[s.upstream]
                        break
                if chosen_bid is not None and chosen_bid in base_pos:
                    focus_xy = base_pos[chosen_bid]

            # If we have a focus point, rotate towards it at the configured speed.
            if focus_xy is not None:
                dx = focus_xy[0] - cx0
                dy = focus_xy[1] - cy0
                if abs(dx) + abs(dy) > 1e-9:
                    desired_deg = math.degrees(math.atan2(dy, dx))
                    max_step = (90.0 / max(1, dv.ticks_per_90)) / subframes
                    div_vis_angle[did] = _step_towards(div_vis_angle[did], desired_deg, max_step)

            ang = math.radians(div_vis_angle[did])
            ln.set_data([cx0, cx0 + 0.5 * math.cos(ang)], [cy0, cy0 + 0.5 * math.sin(ang)])

        # Update belt markers (move continuously)
        speed = 1.0 / subframes
        for i in range(len(belt_markers)):
            belt_markers[i] += speed
        bx, by = compute_belt_marker_xy()
        belt_sc.set_offsets(list(zip(bx, by)))

        # Alarms
        if current_frame.alarms:
            alarm_text.set_text("\n".join(current_frame.alarms[-6:]))
        else:
            alarm_text.set_text("")

        ax.set_title(f"Conveyor Simulator  tick={current_tick}")
        return (base_sc, belt_sc, alarm_text)

    total_frames = max_ticks * subframes
    anim = FuncAnimation(fig, update, frames=total_frames, interval=interval_ms, blit=False, repeat=False)
    plt.show()
