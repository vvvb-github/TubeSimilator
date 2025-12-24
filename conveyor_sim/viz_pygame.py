from __future__ import annotations

"""pygame based realtime visualization.

This module intentionally keeps the same decoupling as the matplotlib viewer:

- The *simulator* (`ConveyorSystem`) owns all state and tick progression.
- The *scheduler* decides actions from a `Snapshot` every tick.
- The *viewer* only renders frames and calls `system.snapshot()` / `system.step()`.

The visual style is inspired by the single-file pygame demo you provided:
dark background, thin grid, track direction hints, station frames, diverter disks
with an indicator line, and bases as circles with a colored "tube" dot.
"""

from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional, Protocol, Deque
import math
import time
from collections import deque


from .sim import ConveyorSystem, FrameData
from .models import Snapshot, Action


class Scheduler(Protocol):
    def decide(self, snap: Snapshot) -> List[Action]: ...


Coord = Tuple[int, int]  # (row, col)


@dataclass
class PygameVizConfig:
    cell_size: int = 40
    margin: int = 50
    log_height: int = 120
    marker_count_per_loop: int = 20


def run_visualization_pygame(
    system: ConveyorSystem,
    scheduler: Scheduler,
    *,
    max_ticks: int = 500,
    tick_ms: int = 400,
    fps: int = 60,
    cfg: Optional[PygameVizConfig] = None,
    window_title: str = "Conveyor Simulator (pygame)",
) -> None:
    """Run a realtime pygame animation.

    Parameters
    ----------
    system:
        The simulator instance.
    scheduler:
        Any object implementing `decide(snapshot) -> List[Action]`.
    max_ticks:
        Max simulation ticks to run (close window earlier to stop).
    tick_ms:
        Real-time duration of one simulation tick.
    fps:
        Target frames-per-second for rendering.
    cfg:
        Visual config (cell size, margins, ...).
    """

    try:
        import pygame  # type: ignore
    except Exception as e:  # pragma: no cover
        raise RuntimeError(
            "pygame is required for the pygame visualization. "
            "Install it with: pip install pygame"
        ) from e

    cfg = cfg or PygameVizConfig()
    cell = cfg.cell_size
    margin = cfg.margin
    log_h = cfg.log_height

    pygame.init()

    grid = system.grid

    screen_w = grid.width * cell + margin * 2
    screen_h = grid.height * cell + margin * 2 + log_h
    screen = pygame.display.set_mode((screen_w, screen_h))
    pygame.display.set_caption(window_title)

    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Arial", 14)
    font_small = pygame.font.SysFont("Arial", 12)

    tick_duration = max(1, tick_ms) / 1000.0

    # Precompute direction hints for each track cell based on loop ordering.
    dir_hint: Dict[Coord, Tuple[int, int]] = {}
    for loop in system.topo.loops:
        L = len(loop)
        for i, rc in enumerate(loop):
            r, c = rc
            r2, c2 = loop[(i + 1) % L]
            dir_hint[rc] = (c2 - c, r2 - r)  # (dx, dy) in cell coords

    # Belt markers (a simple illusion of belt motion)
    loops = system.topo.loops
    belt_s: List[float] = []
    belt_lid: List[int] = []
    for lid, loop in enumerate(loops):
        L = len(loop)
        for k in range(cfg.marker_count_per_loop):
            belt_s.append(k * (L / cfg.marker_count_per_loop))
            belt_lid.append(lid)

    # Visual state
    div_vis_angle: Dict[str, float] = {did: dv.angle_deg for did, dv in system.diverters.items()}
    alarms_history: Deque[str] = deque(maxlen=8)

    base_ids = sorted(system.bases.keys())

    # Tick/animation state
    frame: Optional[FrameData] = None
    tick_start_time = time.time()
    last_render_time = tick_start_time
    paused = False

    def _cell_center_px(x_cell: float, y_cell: float) -> Tuple[float, float]:
        """cell coords (col,row) -> pixel coords (center)."""
        return (
            margin + x_cell * cell + cell / 2.0,
            margin + y_cell * cell + cell / 2.0,
        )

    def _coord_center_px(rc: Coord) -> Tuple[float, float]:
        r, c = rc
        return _cell_center_px(float(c), float(r))

    def _wrap_angle_deg(a: float) -> float:
        return (a + 180.0) % 360.0 - 180.0

    def _step_towards_deg(current: float, target: float, max_step: float) -> float:
        diff = _wrap_angle_deg(target - current)
        if diff > max_step:
            diff = max_step
        elif diff < -max_step:
            diff = -max_step
        return current + diff

    def _tube_color_for_base(bid: str) -> Optional[Tuple[int, int, int]]:
        b = system.bases[bid]
        if b.tube_id is None:
            return None
        tube = system.tubes[b.tube_id]
        if tube.is_done:
            return (0, 255, 0)
        # In-testing?
        in_test = any(
            st.is_busy() and st.busy_base_id == bid and st.busy_op == "test" for st in system.stations.values()
        )
        if in_test:
            return (255, 255, 0)
        return (0, 255, 255)

    def _is_diverter_locked(did: str) -> bool:
        # Visual lock = station currently operating with this diverter as stopper.
        for st in system.stations.values():
            if st.is_busy() and st.stop_diverter_id == did:
                return True
        return False

    running = True
    while running:
        now = time.time()
        dt_frame = now - last_render_time
        last_render_time = now

        # ---- events ----
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_n:
                    # single-step when paused
                    if paused and frame is not None and frame.tick < max_ticks:
                        snap = system.snapshot()
                        acts = scheduler.decide(snap)
                        frame = system.step(acts)
                        if frame.alarms:
                            alarms_history.extend(frame.alarms)
                        tick_start_time = now

        # ---- tick update ----
        if frame is None:
            snap = system.snapshot()
            acts = scheduler.decide(snap)
            frame = system.step(acts)
            if frame.alarms:
                alarms_history.extend(frame.alarms)
            tick_start_time = now

        if not paused:
            elapsed = now - tick_start_time
            if elapsed >= tick_duration:
                if frame.tick >= max_ticks - 1:
                    # Stop automatically after max_ticks
                    running = False
                    continue
                snap = system.snapshot()
                acts = scheduler.decide(snap)
                frame = system.step(acts)
                if frame.alarms:
                    alarms_history.extend(frame.alarms)
                tick_start_time = now
                elapsed = 0.0
        else:
            elapsed = now - tick_start_time

        progress = 0.0
        if tick_duration > 1e-9:
            progress = max(0.0, min(1.0, elapsed / tick_duration))

        # ---- animated base positions (cell coords) ----
        # Frame stores (x=col, y=-row)
        base_pos: Dict[str, Tuple[float, float]] = {}
        if frame is not None:
            for bid in base_ids:
                x0, y0 = frame.base_start_xy[bid]
                x1, y1 = frame.base_end_xy[bid]
                x = x0 + progress * (x1 - x0)
                y = y0 + progress * (y1 - y0)
                base_pos[bid] = (x, -y)  # -> (col, row)

        # coord->base for current tick (for idle diverter focus)
        coord_to_base: Dict[Coord, str] = {}
        for bid, b in system.bases.items():
            if b.coord is not None:
                coord_to_base[b.coord] = bid

        # ---- belt markers update ----
        speed = 0.0 if paused else (dt_frame / tick_duration if tick_duration > 1e-9 else 0.0)
        for i in range(len(belt_s)):
            lid = belt_lid[i]
            L = len(loops[lid])
            belt_s[i] = (belt_s[i] + speed) % max(1, L)

        # ---- diverter indicator angles (smooth aiming) ----
        for did, dv in system.diverters.items():
            # Center of the disk in cell coords
            r1, c1 = dv.side_a.c_coord
            r2, c2 = dv.side_b.c_coord
            cx_cell = (c1 + c2) / 2.0
            cy_cell = (r1 + r2) / 2.0

            focus: Optional[Tuple[float, float]] = None

            # If carrying, follow base; if base held at center, aim at destination slot.
            if dv.cargo_base_id is not None and dv.cargo_base_id in base_pos:
                bxy = base_pos[dv.cargo_base_id]
                if abs(bxy[0] - cx_cell) + abs(bxy[1] - cy_cell) < 1e-6 and dv.cargo_dst is not None:
                    dr, dc = dv.cargo_dst
                    focus = (float(dc), float(dr))
                else:
                    focus = bxy

            # Idle: aim at an upstream waiting base (inner-first)
            if focus is None:
                sides = [dv.side_a, dv.side_b]
                sides.sort(key=lambda s: 0 if s.loop_id != system.outer_loop_id else 1)
                chosen_bid: Optional[str] = None
                for s in sides:
                    if s.upstream in coord_to_base:
                        chosen_bid = coord_to_base[s.upstream]
                        break
                if chosen_bid is not None and chosen_bid in base_pos:
                    focus = base_pos[chosen_bid]

            if focus is not None:
                dx = focus[0] - cx_cell
                dy = focus[1] - cy_cell
                if abs(dx) + abs(dy) > 1e-9:
                    desired_deg = math.degrees(math.atan2(dy, dx))
                    deg_per_tick = 90.0 / max(1, dv.ticks_per_90)
                    max_step = deg_per_tick * (dt_frame / tick_duration) if tick_duration > 1e-9 else deg_per_tick
                    div_vis_angle[did] = _step_towards_deg(div_vis_angle.get(did, 0.0), desired_deg, max_step)

        # ---- render ----
        screen.fill((30, 30, 30))

        # grid + direction hints + stations + C markers
        for r in range(grid.height):
            for c in range(grid.width):
                tok = grid.cells[r][c]
                rect = pygame.Rect(margin + c * cell, margin + r * cell, cell, cell)
                pygame.draw.rect(screen, (50, 50, 50), rect, 1)

                if tok in ("=", "|", "S", "I", "C"):
                    if (r, c) in dir_hint:
                        dx, dy = dir_hint[(r, c)]
                        cxp, cyp = rect.center
                        end = (cxp + dx * 10, cyp + dy * 10)
                        pygame.draw.line(screen, (90, 90, 90), (cxp, cyp), end, 2)

                # station frames
                if tok in ("S", "I"):
                    color = (100, 200, 255) if tok == "I" else (255, 200, 100)
                    pygame.draw.rect(screen, color, rect.inflate(-10, -10), 2)
                    ch_surf = font.render(tok, True, color)
                    screen.blit(ch_surf, ch_surf.get_rect(center=rect.center))

                # diverter cell marker (two adjacent C cells)
                if tok == "C":
                    pygame.draw.rect(screen, (80, 80, 80), rect.inflate(-12, -12), 1)

        # belt markers
        for i, s in enumerate(belt_s):
            lid = belt_lid[i]
            loop = loops[lid]
            L = len(loop)
            if L <= 0:
                continue
            s_mod = s % L
            i0 = int(math.floor(s_mod))
            frac = s_mod - i0
            r0, c0 = loop[i0]
            r1, c1 = loop[(i0 + 1) % L]
            x_cell = c0 + frac * (c1 - c0)
            y_cell = r0 + frac * (r1 - r0)
            px, py = _cell_center_px(x_cell, y_cell)
            pygame.draw.circle(screen, (120, 120, 120), (int(px), int(py)), 2)

        # diverter disks + indicator line
        for did, dv in system.diverters.items():
            r1, c1 = dv.side_a.c_coord
            r2, c2 = dv.side_b.c_coord
            cx_cell = (c1 + c2) / 2.0
            cy_cell = (r1 + r2) / 2.0
            cx, cy = _cell_center_px(cx_cell, cy_cell)

            pygame.draw.circle(screen, (160, 160, 160), (int(cx), int(cy)), cell // 2 - 2, 2)

            locked = _is_diverter_locked(did)
            color = (255, 80, 80) if locked else (80, 255, 80)
            ang = math.radians(div_vis_angle.get(did, 0.0))
            length = cell / 2.0 - 8
            ex = cx + length * math.cos(ang)
            ey = cy + length * math.sin(ang)
            pygame.draw.line(screen, color, (int(cx), int(cy)), (int(ex), int(ey)), 3)

            # label
            label = font_small.render(did, True, (200, 200, 200))
            screen.blit(label, label.get_rect(center=(int(cx), int(cy) + 18)))

        # bases
        for bid in base_ids:
            if bid not in base_pos:
                continue
            x_cell, y_cell = base_pos[bid]
            px, py = _cell_center_px(x_cell, y_cell)
            pygame.draw.circle(screen, (200, 200, 200), (int(px), int(py)), 12)

            tcol = _tube_color_for_base(bid)
            if tcol is not None:
                pygame.draw.circle(screen, tcol, (int(px), int(py)), 8)
                tube_id = system.bases[bid].tube_id or ""
                txt = tube_id[-2:] if len(tube_id) >= 2 else tube_id
                id_surf = font_small.render(txt, True, (0, 0, 0))
                screen.blit(id_surf, id_surf.get_rect(center=(int(px), int(py))))

        # UI text
        info = (
            f"Tick: {frame.tick if frame else system.tick} | "
            f"Bases: {len(system.bases)} | "
            f"ReadyIn: {len(system.ready_input_tubes)} | "
            f"DoneOut: {len(system.completed_output)}"
        )
        if paused:
            info += " | PAUSED (SPACE resume, N step)"
        screen.blit(font.render(info, True, (255, 255, 255)), (10, screen_h - log_h + 10))

        # alarms/logs
        y0 = screen_h - log_h + 32
        if alarms_history:
            for i, line in enumerate(list(alarms_history)[-6:]):
                surf = font_small.render(line, True, (150, 150, 150))
                screen.blit(surf, (10, y0 + i * 16))

        pygame.display.flip()
        clock.tick(max(1, fps))

    pygame.quit()
